from typing import List, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math


class AnalogVoltageInterpreterNode(Node):
    def __init__(self):
        super().__init__('analog_voltage_interpreter')

        # Parameters
        # 入力
        self.declare_parameter('input_topic', '/ai1616llpe/voltage')
        self.declare_parameter('ai_channels', 16)

        # 物理→論理並べ替え: [L_pos, R_pos, L_pres, R_pres, L_load, R_load]
        # 物理割当: CH01=L_pos, CH02=R_pos, CH05=L_pres, CH04=R_pres, CH07=L_load, CH06=R_load,
        # 供給圧をCH03に追加 流量計の値はCH12
        # index_map = [1,2,5,4,7,6,3]
        self.declare_parameter('index_map', [1, 2, 5, 4, 7, 6, 3])

        # 位置(ワイヤセンサ): 5V = 1000mm
        self.declare_parameter('pos_fullscale_v', 5.0)
        self.declare_parameter('pos_fullscale_mm', 1000.0)

        # 圧力(1-5V → [Rmin, Rmax] kPa)
        self.declare_parameter('pres_vmin', 1.0)
        self.declare_parameter('pres_vmax', 5.0)
        self.declare_parameter('pres_rmin_kpa', 0.0)
        self.declare_parameter('pres_rmax_kpa', 1000.0)

        # ロードセル校正（単一値／フォールバック用）
        # 例: 0N時の電圧[-], 単一ゲイン[N/V]
        self.declare_parameter('load_v_zero_N', 0.0)
        self.declare_parameter('load_gain_N_per_v', 116.6196216)

        # L/R個別（未設定(None)なら上の単一値を使用）
        self.declare_parameter('load_gain_L_N_per_v', None)  # 左用 N/V
        self.declare_parameter('load_gain_R_N_per_v', None)  # 右用 N/V
        self.declare_parameter('load_v_zero_L_N', None)      # 左0N電圧[V]
        self.declare_parameter('load_v_zero_R_N', None)      # 右0N電圧[V]

        # -------- Read parameters --------
        gp = self.get_parameter
        self.input_topic: str = gp('input_topic').value
        self.ai_channels: int = int(gp('ai_channels').value)
        _imap = gp('index_map').value
        self.index_map: List[int] = [int(x) for x in _imap] if isinstance(_imap, (list, tuple)) else [1, 2, 5, 4, 7, 6]

        self.pos_fullscale_v: float = float(gp('pos_fullscale_v').value)
        self.pos_fullscale_mm: float = float(gp('pos_fullscale_mm').value)

        self.pres_vmin: float = float(gp('pres_vmin').value)
        self.pres_vmax: float = float(gp('pres_vmax').value)
        self.pres_rmin: float = float(gp('pres_rmin_kpa').value)
        self.pres_rmax: float = float(gp('pres_rmax_kpa').value)

        # 単一設定（フォールバック）
        self.load_v_zero_N: float = float(gp('load_v_zero_N').value)
        self.load_gain_N_per_v: float = float(gp('load_gain_N_per_v').value)

        # 個別設定（Noneなら単一設定を使用）
        gL = gp('load_gain_L_N_per_v').value
        gR = gp('load_gain_R_N_per_v').value
        zL = gp('load_v_zero_L_N').value
        zR = gp('load_v_zero_R_N').value

        self.load_gain_L: float = float(gL) if gL is not None else self.load_gain_N_per_v
        self.load_gain_R: float = float(gR) if gR is not None else self.load_gain_N_per_v
        self.load_zero_L: float = float(zL) if zL is not None else self.load_v_zero_N
        self.load_zero_R: float = float(zR) if zR is not None else self.load_v_zero_N

        self.publish_kg: bool = bool(gp('publish_kg_topics').value)

        # -------- I/O --------
        self.sub = self.create_subscription(Float32MultiArray, self.input_topic, self._cb_voltage, 10)
        self.pub_pos = self.create_publisher(Float32MultiArray, '/sensors/position_mm', 10)
        
        #軽くするために落とす
        self.pub_pres = self.create_publisher(Float32MultiArray, '/sensors/pressure_kpa', 10)
        self.pub_force = self.create_publisher(Float32MultiArray, '/sensors/force_N', 10)

        # 個別可視化
        self.pub_L_pos = self.create_publisher(Float32, '/sensors/L_pos_mm', 10)
        self.pub_R_pos = self.create_publisher(Float32, '/sensors/R_pos_mm', 10)
        self.pub_L_pres = self.create_publisher(Float32, '/sensors/L_pres_kpa', 10)
        self.pub_R_pres = self.create_publisher(Float32, '/sensors/R_pres_kpa', 10)
        self.pub_L_force_N = self.create_publisher(Float32, '/sensors/L_force_N_raw', 10)
        self.pub_R_force_N = self.create_publisher(Float32, '/sensors/R_force_N_raw', 10)
        self.pub_sub_pres = self.create_publisher(Float32, '/sensors/sup_pres_kpa', 10)

        self.get_logger().info(
            f'[{self.get_name()}] input={self.input_topic}, ai_channels={self.ai_channels}, '
            f'index_map={self.index_map}, pos:{self.pos_fullscale_v}V->{self.pos_fullscale_mm}mm, '
            f'pres:{self.pres_vmin}-{self.pres_vmax}V->{self.pres_rmin}-{self.pres_rmax}kPa, '
            f'load: L(V0={self.load_zero_L:.4f} V, gain={self.load_gain_L:.4f} N/V), '
            f'R(V0={self.load_zero_R:.4f} V, gain={self.load_gain_R:.4f} N/V)'
        )

    # -------- Helpers --------
    def _pos_from_v(self, v: float) -> float:
        if self.pos_fullscale_v <= 1e-9:
            return 0.0
        return (v / self.pos_fullscale_v) * self.pos_fullscale_mm

    def _pres_from_v(self, v: float) -> float:
        denom = max(self.pres_vmax - self.pres_vmin, 1e-9)
        ratio = (v - self.pres_vmin) / denom
        ratio = min(1.0, max(0.0, ratio))
        return ratio * (self.pres_rmax - self.pres_rmin) + self.pres_rmin

    def _forceN_from_v_L(self, v: float) -> float:
        return self.load_gain_L * (v - self.load_zero_L)

    def _forceN_from_v_R(self, v: float) -> float:
        return self.load_gain_R * (v - self.load_zero_R)

    # -------- Callback --------
    def _cb_voltage(self, msg: Float32MultiArray):
        raw = list(msg.data)
        if len(raw) < 7:
            return

        try:
            L_pos_v = float(raw[self.index_map[0]])
            R_pos_v = float(raw[self.index_map[1]])
            L_pres_v = float(raw[self.index_map[2]])
            R_pres_v = float(raw[self.index_map[3]])
            L_load_v = float(raw[self.index_map[4]])
            R_load_v = float(raw[self.index_map[5]])
            Sup_pres_v = float(raw[self.index_map[6]])
        except (IndexError, ValueError):
            self.get_logger().warn(f'index_map out of range or invalid: {self.index_map}, len(raw)={len(raw)}')
            return

        # 電圧から物理量に変換
        L_pos_mm = self._pos_from_v(L_pos_v)
        R_pos_mm = self._pos_from_v(R_pos_v)
        L_pres_kpa = self._pres_from_v(L_pres_v)
        R_pres_kpa = self._pres_from_v(R_pres_v)
        L_force_N = self._forceN_from_v_L(L_load_v)
        R_force_N = self._forceN_from_v_R(R_load_v)
        Sup_pres_kpa = self._pres_from_v(Sup_pres_v)

        # 配列メッセージ
        # m_pos = Float32MultiArray();   m_pos.data   = [L_pos_mm,  R_pos_mm]
        # m_pres = Float32MultiArray();  m_pres.data  = [L_pres_kpa, R_pres_kpa]
        # m_force = Float32MultiArray(); m_force.data = [L_force_N, R_force_N]

        # 個別
        self.pub_L_pos.publish(Float32(data=L_pos_mm))
        self.pub_R_pos.publish(Float32(data=R_pos_mm))
        self.pub_L_pres.publish(Float32(data=L_pres_kpa))
        self.pub_R_pres.publish(Float32(data=R_pres_kpa))
        self.pub_L_force_N.publish(Float32(data=L_force_N))
        self.pub_R_force_N.publish(Float32(data=R_force_N))
        self.pub_sup_pres.publish(Float32(data=Sup_pres_kpa))
def main(args=None):
    rclpy.init(args=args)
    node = AnalogVoltageInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
