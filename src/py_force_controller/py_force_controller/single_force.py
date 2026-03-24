import rclpy
import math
from typing import Optional, List

from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class FirstOrderLPF: 
    def __init__(self, cutoff_hz, dt, x0=0.0):
        import math
        tau = 1.0/ (2.0 * math.pi * max(1e-3, cutoff_hz))
        self.alpha = dt / (tau + dt)
        self.y = x0
    def filt(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y
def clamp(x, a, b): return a if x < a else b if x > b else x

class ForceController(Node):
    def __init__(self):
        super().__init__('force_ctrl')
        # ==== パラメータ ====
        # 制御
        self.declare_parameter('rate_hz', 500.0)
        self.declare_parameter('kp', 0.03)
        self.declare_parameter('ki', 0.00)
        self.declare_parameter('i_limit', 5.0)
        self.declare_parameter('neutral_v', 5.0)
        self.declare_parameter('vmin', 0.0)
        self.declare_parameter('vmax', 10.0)
        # self.declare_parameter('valve_cmd_topic', '/actuators/valve_voltage')
        # valve_topic = self.get_parameter('valve_cmd_topic').get_parameter_value.string_value
        # self.pub_out = self.create_publisher(Float32MultiArray, valve_topic, 10)

        # 入出力の単位
        self.declare_parameter('sensor_is_kgf', True)
        self.declare_parameter('target_value', 20.0)      # ↑に合わせた単位で（kgf or N）

        # フィルタ
        self.declare_parameter('force_lpf_hz', 20.0)

        # ★トピック名
        self.declare_parameter('topic_force_L', '/sensors/L_force_N_raw')
        self.declare_parameter('topic_force_R', '/sensors/R_force_N_raw')
        self.declare_parameter('topic_out', '/actuators/valve_voltage')
        # 目標の可視化は“debug”に出す（他のターゲット生成ノードと衝突しないため）
        self.declare_parameter('topic_target_pub', '/debug/target_force_N')
        # フィルタ後の力を L/R 名で出す
        self.declare_parameter('topic_forceL_N_filt', '/sensors/L_force_N_filtered')
        self.declare_parameter('topic_forceR_N_filt', '/sensors/R_force_N_filtered')

        # ==== 読み込み ====
        p = self.get_parameter
        self.rate_hz   = float(p('rate_hz').value)
        self.kp        = float(p('kp').value)
        self.ki        = float(p('ki').value)
        self.i_limit   = float(p('i_limit').value)
        self.neutral_v = float(p('neutral_v').value)
        self.vmin      = float(p('vmin').value)
        self.vmax      = float(p('vmax').value)

        self.sensor_is_kgf = bool(p('sensor_is_kgf').value)
        self.target_value  = float(p('target_value').value)

        self.dt = 1.0 / self.rate_hz

        fc = float(p('force_lpf_hz').value)
        self.lpfL = FirstOrderLPF(fc, self.dt, 0.0)
        self.lpfR = FirstOrderLPF(fc, self.dt, 0.0)

        # ==== 状態 ====
        self.iL = 0.0
        self.iR = 0.0
        self.forceL_raw = 0.0  # 受信（kgf or N）
        self.forceR_raw = 0.0

        # ==== I/O ====
        topic_force_L = str(p('topic_force_L').value)
        topic_force_R = str(p('topic_force_R').value)
        topic_out     = str(p('topic_out').value)
        topic_target  = str(p('topic_target_pub').value)
        topic_fL_filt = str(p('topic_forceL_N_filt').value)
        topic_fR_filt = str(p('topic_forceR_N_filt').value)

        # 入力は “L/R の荷重（kg）” を読むのが現行
        self.sub_L = self.create_subscription(Float32, topic_force_L, self.cb_left, 10)
        self.sub_R = self.create_subscription(Float32, topic_force_R, self.cb_right, 10)

        # 出力は AO ノード
        self.pub_out = self.create_publisher(Float32MultiArray, topic_out, 10)

        # 目標Nとフィルタ後力N
        self.pub_target_N = self.create_publisher(Float32, topic_target, 10)
        self.pub_forceL_N_filt = self.create_publisher(Float32, topic_fL_filt, 10)
        self.pub_forceR_N_filt = self.create_publisher(Float32, topic_fR_filt, 10)

        self.timer = self.create_timer(self.dt, self.loop)

    def cb_left(self, msg):  self.forceL_raw = float(msg.data)
    def cb_right(self, msg): self.forceR_raw = float(msg.data)

    def loop(self):
        g = 9.80665
        # --- 単位統一（N） ---
        if self.sensor_is_kgf:
            forceL_meas_N = self.forceL_raw * g
            forceR_meas_N = self.forceR_raw * g
            target_N = self.target_value * g
        else:
            forceL_meas_N = self.forceL_raw
            forceR_meas_N = self.forceR_raw
            target_N = self.target_value

        self.pub_target_N.publish(Float32(data=float(target_N)))

        # --- LPF ---
        fL = self.lpfL.filt(forceL_meas_N)
        fR = self.lpfR.filt(forceR_meas_N)
        self.pub_forceL_N_filt.publish(Float32(data=float(fL)))
        self.pub_forceR_N_filt.publish(Float32(data=float(fR)))

        # --- PI（電圧） ---
        eL = target_N - fL
        eR = target_N - fR

        uL_unsat = self.neutral_v + self.kp*eL + self.ki*self.iL
        uR_unsat = self.neutral_v + self.kp*eR + self.ki*self.iR

        uL = clamp(uL_unsat, self.vmin, self.vmax)
        uR = clamp(uR_unsat, self.vmin, self.vmax)

        # AIW: 飽和時は積分停止
        if uL == uL_unsat: self.iL = clamp(self.iL + eL*self.dt, -self.i_limit, self.i_limit)
        if uR == uR_unsat: self.iR = clamp(self.iR + eR*self.dt, -self.i_limit, self.i_limit)

        # --- 出力 publish ---
        msg = Float32MultiArray()
        msg.data = [float(uL), float(uR)]
        self.pub_out.publish(msg)

def main():
    rclpy.init()
    node = ForceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
