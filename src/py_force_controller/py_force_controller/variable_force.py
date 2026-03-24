import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


def clamp(x, a, b): 
    return a if x < a else b if x > b else x


class FirstOrderLPF:
    """一時遅れ IIR ローパス: y[k] = y[k-1] + alpha*(x[k] - y[k-1])"""
    def __init__(self, cutoff_hz: float, dt: float, x0: float = 0.0):
        cutoff_hz = max(1e-3, float(cutoff_hz))
        dt = float(dt)
        tau = 1.0 / (2.0 * math.pi * cutoff_hz)
        self.alpha = dt / (tau + dt)
        self.y = float(x0)

    def filt(self, x: float) -> float:
        self.y += self.alpha * (float(x) - self.y)
        return self.y


class SingleForceController(Node):
    """
    単純な力PI制御（左右独立）＋ 目標力 F_ref = a*x + b（xはワイヤセンサ位置[mm]）
    - 入力:  力（左右） [N または kgf]、位置[mm]
    - 出力:  バルブ指令電圧（左右）[V]  （/actuators/valve_voltage）
    """

    def __init__(self):
        super().__init__('single_force_controller')

        # ---- 基本パラメータ（制御） ----
        self.declare_parameter('rate_hz', 500.0)     # ループ周期
        self.declare_parameter('kp', 0.03)           # 力Pゲイン [V/N]
        self.declare_parameter('ki', 0.00)           # 力Iゲイン [V/(N·s)]
        self.declare_parameter('i_limit', 5.0)       # 積分クランプ [N·s 等価]
        self.declare_parameter('neutral_v', 5.0)     # バルブ中立 [V]
        self.declare_parameter('vmin', 0.1)          # 出力最小 [V]
        self.declare_parameter('vmax', 9.9)          # 出力最大 [V]

        # ---- 力センサ単位・トピック ----
        self.declare_parameter('sensor_is_kgf', False)  # True なら kgf 入力 → Nに換算
        self.declare_parameter('topic_force_L', '/sensors/L_force_N_raw')
        self.declare_parameter('topic_force_R', '/sensors/R_force_N_raw')

        # ---- 位置→目標力（F = a*x + b） ----
        self.declare_parameter('use_affine_target', True)
        self.declare_parameter('pos_topic', '/sensors/L_pos_mm')  # Float32 [mm]
        self.declare_parameter('pos_lpf_hz', 10.0)
        self.declare_parameter('a_N_per_mm', 0.20)   # 傾き N/mm（糸が伸びるほど力↑なら +）
        self.declare_parameter('b_N', 0.0)           # 切片 N
        self.declare_parameter('target_min_N', 0.0)  # 目標力下限
        self.declare_parameter('target_max_N', 500.0)# 目標力上限

        # ---- AO出力トピック ----
        self.declare_parameter('topic_out', '/actuators/valve_voltage')

        # ---- パラメータ読込 ----
        g = 9.80665
        p = self.get_parameter
        self.rate_hz   = float(p('rate_hz').value)
        self.dt        = 1.0 / self.rate_hz
        self.kp        = float(p('kp').value)
        self.ki        = float(p('ki').value)
        self.i_limit   = float(p('i_limit').value)
        self.neutral_v = float(p('neutral_v').value)
        self.vmin      = float(p('vmin').value)
        self.vmax      = float(p('vmax').value)

        self.sensor_is_kgf = bool(p('sensor_is_kgf').value)
        self.topic_force_L = str(p('topic_force_L').value)
        self.topic_force_R = str(p('topic_force_R').value)

        self.use_affine   = bool(p('use_affine_target').value)
        self.pos_topic    = str(p('pos_topic').value)
        self.pos_lpf_hz   = float(p('pos_lpf_hz').value)
        self.a_N_per_mm   = float(p('a_N_per_mm').value)
        self.b_N          = float(p('b_N').value)
        self.target_min_N = float(p('target_min_N').value)
        self.target_max_N = float(p('target_max_N').value)

        self.topic_out = str(p('topic_out').value)

        # ---- フィルタ ----
        self.lpfL = FirstOrderLPF(cutoff_hz=20.0, dt=self.dt, x0=0.0)  # 力フィルタ L
        self.lpfR = FirstOrderLPF(cutoff_hz=20.0, dt=self.dt, x0=0.0)  # 力フィルタ R
        self.lpf_pos = FirstOrderLPF(cutoff_hz=self.pos_lpf_hz, dt=self.dt, x0=0.0)  # 位置フィルタ

        # ---- 内部状態 ----
        self.iL = 0.0
        self.iR = 0.0
        self.forceL_raw = 0.0
        self.forceR_raw = 0.0
        self.pos_mm     = 0.0   # ワイヤセンサ位置[mm]

        # ---- I/O ----
        self.sub_L = self.create_subscription(Float32, self.topic_force_L, self.cb_left, 10)
        self.sub_R = self.create_subscription(Float32, self.topic_force_R, self.cb_right, 10)
        self.sub_pos = self.create_subscription(Float32, self.pos_topic, self.cb_pos, 10)

        self.pub_out = self.create_publisher(Float32MultiArray, self.topic_out, 10)
        self.pub_target_N = self.create_publisher(Float32, '/targets/force_N', 10)
        self.pub_forceL_N_filt = self.create_publisher(Float32, '/debug/L_force_N_filtered', 10)
        self.pub_forceR_N_filt = self.create_publisher(Float32, '/debug/R_force_N_filtered', 10)
        self.pub_pos_mm_filt   = self.create_publisher(Float32, '/debug/pos_mm_filtered', 10)

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f'[single_force_controller] rate={self.rate_hz}Hz, kp={self.kp}, ki={self.ki}, '
            f'neutral={self.neutral_v}V, out=[{self.vmin},{self.vmax}]V, '
            f'pos_topic={self.pos_topic}, F=ax+b: a={self.a_N_per_mm} N/mm, b={self.b_N} N'
        )

        self._g = g  # 係数保持（kgf→N 変換用）

    # ---- Callbacks ----
    def cb_left(self, msg: Float32):
        self.forceL_raw = float(msg.data)

    def cb_right(self, msg: Float32):
        self.forceR_raw = float(msg.data)

    def cb_pos(self, msg: Float32):
        self.pos_mm = float(msg.data)

    # ---- Main loop ----
    def loop(self):
        # 力計を N に統一
        if self.sensor_is_kgf:
            fL_meas_N = self.forceL_raw * self._g
            fR_meas_N = self.forceR_raw * self._g
        else:
            fL_meas_N = self.forceL_raw
            fR_meas_N = self.forceR_raw

        # 位置LPF
        pos_f = self.lpf_pos.filt(self.pos_mm)
        self.pub_pos_mm_filt.publish(Float32(data=float(pos_f)))

        # 目標力：F = a*x + b（必要ならクランプ）
        if self.use_affine:
            target_N = self.a_N_per_mm * pos_f + self.b_N
            target_N = clamp(target_N, self.target_min_N, self.target_max_N)
        else:
            # もし従来の固定Fを残したい場合は、別パラメータ（target_value_N）を追加してここで使う
            target_N = 0.0

        self.pub_target_N.publish(Float32(data=float(target_N)))

        # 測定力にLPF
        fL = self.lpfL.filt(fL_meas_N)
        fR = self.lpfR.filt(fR_meas_N)
        self.pub_forceL_N_filt.publish(Float32(data=float(fL)))
        self.pub_forceR_N_filt.publish(Float32(data=float(fR)))

        # 誤差
        eL = target_N - fL
        eR = target_N - fR

        # PI（アンチワインドアップ：飽和時は積分停止）
        uL_unsat = self.neutral_v + self.kp*eL + self.ki*self.iL
        uR_unsat = self.neutral_v + self.kp*eR + self.ki*self.iR

        uL = clamp(uL_unsat, self.vmin, self.vmax)
        uR = clamp(uR_unsat, self.vmin, self.vmax)

        if uL == uL_unsat:
            self.iL = clamp(self.iL + eL * self.dt, -self.i_limit, self.i_limit)
        if uR == uR_unsat:
            self.iR = clamp(self.iR + eR * self.dt, -self.i_limit, self.i_limit)

        # 出力
        msg = Float32MultiArray()
        msg.data = [float(uL), float(uR)]
        self.pub_out.publish(msg)


def main():
    rclpy.init()
    node = SingleForceController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
