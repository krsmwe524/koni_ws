import rclpy
import math
from typing import Optional, List

from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

def clamp(x, a, b): return a if x < a else b if x > b else x

class FirstOrderLPF: #IIR/将来的に川嶋先生がスラックで言ってたフィルタにしたほうがいいかも。今は大野さんのに準拠
    def __init__(self, cutoff_hz, dt, x0=0.0):
        import math
        tau = 1.0/ (2.0 * math.pi * max(1e-3, cutoff_hz))
        self.alpha = dt / (tau + dt)
        self.y = x0
    def filt(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y

class ButterworthLPF2:
    """
    デジタル2次バターワースLPF
    係数はTPT（tan法）で計算：K = tan(pi*fc/fs)
    y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    """
    def __init__(self, cutoff_hz, dt, x0=0.0):
        import math
        fs = 1.0 / max(1e-9, dt)
        fc = max(1e-3, cutoff_hz)
        # Butterworth 2次 → Q = 1/sqrt(2)
        Q = 1.0 / math.sqrt(2.0)
        K = math.tan(math.pi * fc / fs)
        KK = K * K
        norm = 1.0 / (1.0 + K/Q + KK)
        self.b0 = KK * norm
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (KK - 1.0) * norm
        self.a2 = (1.0 - K/Q + KK) * norm

        self.x1 = x0
        self.x2 = x0
        self.y1 = x0
        self.y2 = x0

    def filt(self, x):
        y = (self.b0 * x +
             self.b1 * self.x1 +
             self.b2 * self.x2 -
             self.a1 * self.y1 -
             self.a2 * self.y2)

        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

class ForceController(Node):
    def __init__(self):
        super().__init__('force_ctrl')
        self.pub_forceL_N_filt = self.create_publisher(Float32, '/sensors/left_force_N_filtered', 10)
        self.pub_forceR_N_filt = self.create_publisher(Float32, '/sensors/right_force_N_filtered', 10)
        # パラメータ
        self.declare_parameter('rate_hz', 500.0)
        self.declare_parameter('kp', 0.03)  
        self.declare_parameter('ki', 0.00)   
        self.declare_parameter('neutral_v', 5.0)
        self.declare_parameter('vmin', 0.0)
        self.declare_parameter('vmax', 10.0)
        self.declare_parameter('sensor_is_kgf', True)
        self.declare_parameter('target_value', 20.0)  # 入力単位はsensor_is_kgfに合わせる→直感的にわかりやすいからkgfかな？
        self.declare_parameter('force_lpf_hz', 20.0) #カットオフは20Hzで
        self.declare_parameter('i_limit', 5.0)  # 積分クランプ

        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        self.kp        = float(self.get_parameter('kp').value) 
        self.ki        = float(self.get_parameter('ki').value)
        self.neutral_v = float(self.get_parameter('neutral_v').value)
        self.vmin      = float(self.get_parameter('vmin').value)
        self.vmax      = float(self.get_parameter('vmax').value)
        self.sensor_is_kgf = bool(self.get_parameter('sensor_is_kgf').value)
        self.target_value  = float(self.get_parameter('target_value').value)
        self.i_limit   = float(self.get_parameter('i_limit').value)

        self.dt = 1.0 / self.rate_hz
        g = 9.80665

        #LPF(カットオフは20Hzで)
        fc = float(self.get_parameter('force_lpf_hz').value)
        # self.lpfL = FirstOrderLPF(fc, self.dt, 0.0)
        # self.lpfR = FirstOrderLPF(fc, self.dt, 0.0)
        self.lpfL = FirstOrderLPF(fc, self.dt, 0.0)
        self.lpfR = FirstOrderLPF(fc, self.dt, 0.0)

        # 状態
        self.iL = 0.0
        self.iR = 0.0
        self.forceL_raw = 0.0  # 購読コールバックで更新(kgfかN)
        self.forceR_raw = 0.0

        #I/O・タイマ
        self.sub_L = self.create_subscription(Float32, '/sensors/left_load',  self.cb_left,  10)
        self.sub_R = self.create_subscription(Float32, '/sensors/right_load', self.cb_right, 10)
        self.pub_out = self.create_publisher(Float32MultiArray, '/ao/cmd_voltages', 10)
        self.timer = self.create_timer(self.dt, self.loop)
        self.pub_target_N = self.create_publisher(Float32, '/sensors/target_force_N', 10)

    def cb_left(self, msg):  self.forceL_raw = float(msg.data)
    def cb_right(self, msg): self.forceR_raw = float(msg.data)

    def loop(self):
        g = 9.80665
        # Nに統一
        if self.sensor_is_kgf:
            forceL_meas_N = self.forceL_raw * g
            forceR_meas_N = self.forceR_raw * g
            target_N = self.target_value * g   # target_valueがkgf入力の時にgをかける
        else:
            forceL_meas_N = self.forceL_raw
            forceR_meas_N = self.forceR_raw
            target_N = self.target_value  # target_valueがNならそのまま
        
    
        self.pub_target_N.publish(Float32(data=float(target_N)))

        # LPFかける。
        fL = self.lpfL.filt(forceL_meas_N)
        fR = self.lpfR.filt(forceR_meas_N)
        # publish!
        self.pub_forceL_N_filt.publish(Float32(data=float(fL)))
        self.pub_forceR_N_filt.publish(Float32(data=float(fR)))

        #誤差項
        eL = target_N - fL
        eR = target_N - fR

        # 出力(V)PI
        uL_unsat = self.neutral_v + self.kp*eL + self.ki*self.iL
        uR_unsat = self.neutral_v + self.kp*eR + self.ki*self.iR

        #飽和
        uL = max(self.vmin, min(self.vmax, uL_unsat))
        uR = max(self.vmin, min(self.vmax, uR_unsat))

        #アンチワインドアップ：飽和時は積分停止
        if uL == uL_unsat:
            self.iL += eL * self.dt
        if uR == uR_unsat:
            self.iR += eR * self.dt

        #積分クランプ
        self.iL = max(-self.i_limit, min(self.i_limit, self.iL))
        self.iR = max(-self.i_limit, min(self.i_limit, self.iR))

        # publish!
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

class ForceTargetGenerator(Node):
    """
    /targets/force_N を配信。
    将来: mode='height_prop' や 'random' 等を追加しやすい構造。
    """
    def __init__(self):
        super().__init__('force_target_generator')
        self.declare_parameter('mode', 'constant')     # 'constant' ほか将来拡張
        self.declare_parameter('constant_N', 200.0)
        self.declare_parameter('rate_hz', 200.0)
        self.mode = self.get_parameter('mode').value
        self.constant_N = float(self.get_parameter('constant_N').value)
        self.dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.pub = self.create_publisher(Float32, '/targets/force_N', 10)
        self.timer = self.create_timer(self.dt, self.loop)

    def loop(self):
        # 今は定常のみ
        self.pub.publish(Float32(data=float(self.constant_N)))

# 外側ループ：力→圧力（PI + フィードフォワード)
class ForceOuterController(Node):
    """
    入力:
      /targets/force_N (Float32) : 目標力[N]
      力センサ:   /sensors/L_force_N_raw or /sensors/L_force_N_filtered（R も同様）
      位置（mm）: /sensors/L_pos_mm, /sensors/R_pos_mm
    出力:
      /targets/pressure_kpa (Float32MultiArray [pL_ref, pR_ref])
    """
    def __init__(self):
        super().__init__('force_outer')

        # 調整項
        self.declare_parameter('ff_alpha', 1.0)
        self.declare_parameter('ff_b0', 0.0)
        self.declare_parameter('ff_b1', 0.0)
        #self.declare_parameter('ff_beta', 0.0)

        self.declare_parameter('use_filtered_force', False)
        self.declare_parameter('force_topic_L_raw',  '/sensors/L_force_N_raw')
        self.declare_parameter('force_topic_R_raw',  '/sensors/R_force_N_raw')
        self.declare_parameter('force_topic_L_filt', '/sensors/L_force_N_filtered')
        self.declare_parameter('force_topic_R_filt', '/sensors/R_force_N_filtered')
        self.declare_parameter('pos_topic_L_mm', '/sensors/L_pos_mm')
        self.declare_parameter('pos_topic_R_mm', '/sensors/R_pos_mm')
        self.declare_parameter('pos_lpf_hz', 2.0) #ワイヤセンサのフィルタ、結構強め。
        # ゲイン
        self.declare_parameter('rate_hz', 500.0)
        self.declare_parameter('F_lpf_hz', 20.0)   # raw使用時の保険LPF
        self.declare_parameter('kp_F', 1.0)        # [kPa/N]
        self.declare_parameter('ki_F', 0.0)        # [kPa/(N·s)]
        self.declare_parameter('i_limit_F', 500.0) # [kPa] 積分クランプ
        self.declare_parameter('pmin_kpa', 0.0)
        self.declare_parameter('pmax_kpa', 600.0)
        # pamの諸々の値
        self.declare_parameter('theta0_deg', 25.0) 
        self.declare_parameter('init_length_mm', 1500.0)
        self.declare_parameter('D0_mm', 40.0)

        # 長さLの近似: L = top_mm - pos_mm
        self.declare_parameter('top_L_mm', 1600.0)
        self.declare_parameter('top_R_mm', 1600.0)

        # 読み込み
        p = self.get_parameter
        self.ff_alpha = float(p('ff_alpha').value)
        #self.ff_beta = float(p('ff_beta').value)
        self.ff_b0 = float(p('ff_b0').value)
        self.ff_b1 = float(p('ff_b1').value)

        self.use_filtered = bool(p('use_filtered_force').value)
        self.dt = 1.0 / float(p('rate_hz').value)
        self.kp_F = float(p('kp_F').value)
        self.ki_F = float(p('ki_F').value)
        self.i_limit_F = float(p('i_limit_F').value)
        self.pmin = float(p('pmin_kpa').value)
        self.pmax = float(p('pmax_kpa').value)
        self.top_L_mm = float(p('top_L_mm').value)
        self.top_R_mm = float(p('top_R_mm').value)

        self.D0_m = float(self.get_parameter('D0_mm').value)/1000.0
        self.theta0 = math.radians(float(self.get_parameter('theta0_deg').value))
        self.L0_m = float(self.get_parameter('init_length_mm').value)/1000.0
        fc_pos = float(self.get_parameter('pos_lpf_hz').value)
        self.posL_filt = FirstOrderLPF(fc_pos, self.dt, 0.0)
        self.posR_filt = FirstOrderLPF(fc_pos, self.dt, 0.0)
        self.pub_pff = self.create_publisher(Float32MultiArray, '/debug/p_ff_kpa', 10)
        self.pub_L_pos_mm_filt = self.create_publisher(Float32, '/debug/L_pos_mm_filtered', 10)
        self.pub_R_pos_mm_filt = self.create_publisher(Float32, '/debug/R_pos_mm_filtered', 10)
        self.pub_len = self.create_publisher(Float32MultiArray, '/debug/pam_length_mm', 10)
        self.pub_L_len = self.create_publisher(Float32, '/debug/L_pam_length_mm', 10)
        self.pub_R_len = self.create_publisher(Float32, '/debug/R_pam_length_mm', 10)


        #状態
        self.Fref = 0.0
        self.FL = 0.0; self.FR = 0.0
        self.posL_mm = 0.0; self.posR_mm = 0.0
        self.iL = 0.0; self.iR = 0.0

        # LPF（raw使用時だけ効く）
        self.lpfL = FirstOrderLPF(float(p('F_lpf_hz').value), self.dt, 0.0)
        self.lpfR = FirstOrderLPF(float(p('F_lpf_hz').value), self.dt, 0.0)

        # I/O
        topic_FL = p('force_topic_L_filt').value if self.use_filtered else p('force_topic_L_raw').value
        topic_FR = p('force_topic_R_filt').value if self.use_filtered else p('force_topic_R_raw').value
        self.sub_FL   = self.create_subscription(Float32, topic_FL, self.cb_FL, 10)
        self.sub_FR   = self.create_subscription(Float32, topic_FR, self.cb_FR, 10)
        self.sub_Fref = self.create_subscription(Float32, '/targets/force_N', self.cb_Fref, 10)
        self.sub_posL = self.create_subscription(Float32, p('pos_topic_L_mm').value, self.cb_posL, 10)
        self.sub_posR = self.create_subscription(Float32, p('pos_topic_R_mm').value, self.cb_posR, 10)
        self.pub_pref = self.create_publisher(Float32MultiArray, '/targets/pressure_kpa', 10)
        self.timer = self.create_timer(self.dt, self.loop)

    # --- コールバック ---
    def cb_FL(self, msg: Float32):
        val = float(msg.data)
        self.FL = self.lpfL.filt(val) if not self.use_filtered else val
    def cb_FR(self, msg: Float32):
        val = float(msg.data)
        self.FR = self.lpfR.filt(val) if not self.use_filtered else val
    def cb_Fref(self, msg: Float32): self.Fref = float(msg.data)
    def cb_posL(self, msg: Float32): self.posL_mm = float(msg.data)
    def cb_posR(self, msg: Float32): self.posR_mm = float(msg.data)

    #長さmm上端高さ
    def pam_length_mm(self, side: str) -> float:
        if side == 'L':
            return max(1.0, self.top_L_mm - self.posL_mm)
        else:
            return max(1.0, self.top_R_mm - self.posR_mm)

    # フィードフォワード項p_ff(F, L) kPa Chouモデル
    def p_ff_kpa_side(self, F_N: float, L_mm: float) -> float:

        L = max(1e-6, L_mm/1000.0)
        #収縮率
        eps = clamp((self.L0_m - L) / max(self.L0_m, 1e-6), -0.5, 0.9)
        # 係数
        cos0 = math.cos(self.theta0)
        sin0 = math.sin(self.theta0)
        term = 3.0 * (1.0 - eps)**2 * (cos0*cos0) - 1.0
        term = max(term, 1e-6)  # 過収縮ガード←いる？
        b = (math.pi/4.0) * (self.D0_m**2) * (1.0/(sin0*sin0)) * term 
        p_pa = F_N / max(b, 1e-9)  #Pa
        p_kpa = p_pa / 1000.0

        # Chou Modelと実測値のズレ。イプシロンによって差がある
        beta = self.ff_b0 + self.ff_b1 * eps

        p_kpa = self.ff_alpha * p_kpa + beta
        return clamp(p_kpa, self.pmin, self.pmax)
        #return 0 #¥フィードフォワード項なくしてみる

    def loop(self):
        Fref = self.Fref
        posL_f = self.posL_filt.filt(self.posL_mm)
        posR_f = self.posR_filt.filt(self.posR_mm)

        self.pub_L_pos_mm_filt.publish(Float32(data=float(posL_f)))
        self.pub_R_pos_mm_filt.publish(Float32(data=float(posR_f)))

        L_L = max(1.0, self.top_L_mm - posL_f)
        L_R = max(1.0, self.top_R_mm - posR_f)
        
        mlen = Float32MultiArray(); mlen.data = [float(L_L), float(L_R)]
        self.pub_len.publish(mlen)
        self.pub_L_len.publish(Float32(data=float(L_L)))
        self.pub_R_len.publish(Float32(data=float(L_R)))

        pff_L = self.p_ff_kpa_side(Fref, L_L)
        pff_R = self.p_ff_kpa_side(Fref, L_R)

        mff = Float32MultiArray(); mff.data = [float(pff_L), float(pff_R)]
        self.pub_pff.publish(mff)
        # 誤差と積分
        eL = Fref - self.FL
        eR = Fref - self.FR
        self.iL = clamp(self.iL + eL * self.dt, -self.i_limit_F, self.i_limit_F)
        self.iR = clamp(self.iR + eR * self.dt, -self.i_limit_F, self.i_limit_F)

        # PI + FF → p_ref
        pL = clamp(pff_L + self.kp_F*eL + self.ki_F*self.iL, self.pmin, self.pmax)
        pR = clamp(pff_R + self.kp_F*eR + self.ki_F*self.iR, self.pmin, self.pmax)

        msg = Float32MultiArray(); msg.data = [float(pL), float(pR)]
        self.pub_pref.publish(msg)

#内側：圧力→電圧PI
class PressureInnerController(Node):
    """
    入力:
      /targets/pressure_kpa [pL_ref, pR_ref]
      /sensors/L_pres_kpa(_filtered), /sensors/R_pres_kpa(_filtered)
    出力:
      /actuators/valve_voltage [uL, uR] (0–10V)
    """
    def __init__(self):
        super().__init__('pressure_inner')
        # 入力トピック
        self.declare_parameter('pres_topic_L', '/sensors/L_pres_kpa_filtered')
        self.declare_parameter('pres_topic_R', '/sensors/R_pres_kpa_filtered')
        self.declare_parameter('pref_topic',   '/targets/pressure_kpa')


        # ループ
        self.declare_parameter('rate_hz', 1000.0)
        self.declare_parameter('P_lpf_hz', 40.0)
        self.declare_parameter('kp_P', 0.01)     # [V/kPa]
        self.declare_parameter('ki_P', 0.00)     # [V/(kPa·s)]
        self.declare_parameter('i_limit_P', 5.0) # 積分クランプ量（V換算）

        # 出力トピック（AOが購読）
        self.declare_parameter('valve_cmd_topic', '/actuators/valve_voltage')
        # 出力レンジ
        self.declare_parameter('neutral_v', 5.0)
        self.declare_parameter('vmin', 0.1)
        self.declare_parameter('vmax', 9.9)

        p = self.get_parameter
        self.dt = 1.0 / float(p('rate_hz').value)
        self.kp = float(p('kp_P').value)
        self.ki = float(p('ki_P').value)
        self.i_limit = float(p('i_limit_P').value)
        self.neutral = float(p('neutral_v').value)
        self.vmin = float(p('vmin').value); self.vmax = float(p('vmax').value)

        # LPF（測定圧の軽いフィルタ）
        self.lpfL = FirstOrderLPF(float(p('P_lpf_hz').value), self.dt, 0.0)
        self.lpfR = FirstOrderLPF(float(p('P_lpf_hz').value), self.dt, 0.0)

        # 状態
        self.pL = 0.0; self.pR = 0.0
        self.pLref = 0.0; self.pRref = 0.0
        self.iL = 0.0; self.iR = 0.0

        # I/O
        self.sub_PL = self.create_subscription(Float32, p('pres_topic_L').value, self.cb_PL, 10)
        self.sub_PR = self.create_subscription(Float32, p('pres_topic_R').value, self.cb_PR, 10)
        self.sub_pref = self.create_subscription(Float32MultiArray, p('pref_topic').value, self.cb_pref, 10)
        self.pub_out = self.create_publisher(Float32MultiArray, p('valve_cmd_topic').value, 10)
        self.pub_PL_f = self.create_publisher(Float32, '/sensors/L_pres_kpa_filtered', 10)
        self.pub_PR_f = self.create_publisher(Float32, '/sensors/R_pres_kpa_filtered', 10)

        self.timer = self.create_timer(self.dt, self.loop)

    def cb_PL(self, msg: Float32):
        y = self.lpfL.filt(float(msg.data))
        self.pL = y
        self.pub_PL_f.publish(Float32(data=float(y)))

    def cb_PR(self, msg: Float32):
        y = self.lpfR.filt(float(msg.data))
        self.pR = y
        self.pub_PR_f.publish(Float32(data=float(y)))

    def cb_pref(self, msg: Float32MultiArray):
        arr = list(msg.data) if msg.data else [0.0, 0.0]
        self.pLref = float(arr[0]); self.pRref = float(arr[1])

    def loop(self):
        # 左
        eL = self.pLref - self.pL
        uL_unsat = self.neutral + self.kp*eL + self.ki*self.iL
        uL = clamp(uL_unsat, self.vmin, self.vmax)
        if uL == uL_unsat:  # 飽和していない時のみ積分
            self.iL = clamp(self.iL + eL * self.dt, -self.i_limit, self.i_limit)

        # 右
        eR = self.pRref - self.pR
        uR_unsat = self.neutral + self.kp*eR + self.ki*self.iR
        uR = clamp(uR_unsat, self.vmin, self.vmax)
        if uR == uR_unsat:
            self.iR = clamp(self.iR + eR * self.dt, -self.i_limit, self.i_limit)

        msg = Float32MultiArray(); msg.data = [float(uL), float(uR)]
        self.pub_out.publish(msg)

# main：同一実行ファイルから役割を切替
def main():
    import sys
    rclpy.init()
    # 引数でノード種別を指定（例：--node-kind target|outer|inner|all）
    node_kind = 'all'
    if '--node-kind' in sys.argv:
        i = sys.argv.index('--node-kind')
        if i + 1 < len(sys.argv):
            node_kind = sys.argv[i + 1].lower()

    nodes = []
    try:
        if node_kind in ('target', 'all'):
            nodes.append(ForceTargetGenerator())
        if node_kind in ('outer', 'all'):
            nodes.append(ForceOuterController())
        if node_kind in ('inner', 'all'):
            nodes.append(PressureInnerController())

        if len(nodes) == 1:
            rclpy.spin(nodes[0])
        else:
            from rclpy.executors import MultiThreadedExecutor
            execu = MultiThreadedExecutor()
            for n in nodes: execu.add_node(n)
            execu.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

from std_msgs.msg import Float32, Float32MultiArray
from rclpy.node import Node

def clamp(x, a, b): return a if x < a else b if x > b else x

class FirstOrderLPF:
    def __init__(self, cutoff_hz, dt, x0=0.0):
        import math
        tau = 1.0 / (2.0 * math.pi * max(1e-3, cutoff_hz))
        self.alpha = dt / (tau + dt)
        self.y = x0
    def filt(self, x):
        self.y += self.alpha * (x - self.y)
        return self.y
