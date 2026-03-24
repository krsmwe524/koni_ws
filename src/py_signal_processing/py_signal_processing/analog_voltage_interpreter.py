from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

class AnalogVoltageInterpreterNode(Node):
    def __init__(self):
        super().__init__('analog_voltage_interpreter')

        # Parameters
        self.declare_parameter('input_topic', '/ai1616llpe/voltage')
        self.declare_parameter('ai_channels', 16) # 16チャンネルに更新

        # 物理→論理並べ替え: [L_pos, R_pos, L_pres, R_pres, L_load, R_load]
        self.declare_parameter('index_map', [1, 2, 5, 4, 7, 6])

        # 位置・圧力の設定
        self.declare_parameter('pos_fullscale_v', 5.0)
        self.declare_parameter('pos_fullscale_mm', 1000.0)
        self.declare_parameter('pres_vmin', 1.0)
        self.declare_parameter('pres_vmax', 5.0)
        self.declare_parameter('pres_rmin_kpa', 0.0)
        self.declare_parameter('pres_rmax_kpa', 1000.0)

        # ロードセル校正（警告回避のためデフォルト値を 0.0 や具体値に設定）
        self.declare_parameter('load_v_zero_N', 0.0)
        self.declare_parameter('load_gain_N_per_v', 116.6196)
        self.declare_parameter('load_gain_L_N_per_v', 0.0)  # None ではなく 0.0
        self.declare_parameter('load_gain_R_N_per_v', 0.0)
        self.declare_parameter('load_v_zero_L_N', 0.0)
        self.declare_parameter('load_v_zero_R_N', 0.0)

        self.declare_parameter('publish_kg_topics', False)

        # -------- Read parameters --------
        gp = self.get_parameter
        self.input_topic = gp('input_topic').value
        self.ai_channels = int(gp('ai_channels').value)
        _imap = gp('index_map').value
        self.index_map = [int(x) for x in _imap]

        self.pos_fullscale_v = float(gp('pos_fullscale_v').value)
        self.pos_fullscale_mm = float(gp('pos_fullscale_mm').value)
        self.pres_vmin = float(gp('pres_vmin').value)
        self.pres_vmax = float(gp('pres_vmax').value)
        self.pres_rmin = float(gp('pres_rmin_kpa').value)
        self.pres_rmax = float(gp('pres_rmax_kpa').value)

        # 共通デフォルト
        d_v_zero = float(gp('load_v_zero_N').value)
        d_gain = float(gp('load_gain_N_per_v').value)

        # 個別設定の読み込み（0.0 の場合は共通デフォルトを使用）
        gL = float(gp('load_gain_L_N_per_v').value)
        gR = float(gp('load_gain_R_N_per_v').value)
        zL = float(gp('load_v_zero_L_N').value)
        zR = float(gp('load_v_zero_R_N').value)

        self.load_gain_L = gL if gL != 0.0 else d_gain
        self.load_gain_R = gR if gR != 0.0 else d_gain
        self.load_zero_L = zL if zL != 0.0 else d_v_zero
        self.load_zero_R = zR if zR != 0.0 else d_v_zero

        self.publish_kg = bool(gp('publish_kg_topics').value)

        # -------- I/O --------
        self.sub = self.create_subscription(Float32MultiArray, self.input_topic, self._cb_voltage, 10)

        # パブリッシャー
        self.pub_L_pos = self.create_publisher(Float32, '/sensors/L_pos_mm', 10)
        self.pub_R_pos = self.create_publisher(Float32, '/sensors/R_pos_mm', 10)
        self.pub_L_pres = self.create_publisher(Float32, '/sensors/L_pres_kpa', 10)
        self.pub_R_pres = self.create_publisher(Float32, '/sensors/R_pres_kpa', 10)
        
        # --- ★ 追加: 供給圧のパブリッシャー ---
        self.pub_sup_pres = self.create_publisher(Float32, '/sensors/sup_press', 10)
        
        self.pub_L_force_N = self.create_publisher(Float32, '/sensors/L_force_N_raw', 10)
        self.pub_R_force_N = self.create_publisher(Float32, '/sensors/R_force_N_raw', 10)

        self.get_logger().info(f'[{self.get_name()}] 供給圧(/sensors/sup_press)の追加と警告修正完了')

    # -------- Helpers --------
    def _pos_from_v(self, v: float) -> float:
        if self.pos_fullscale_v <= 1e-9: return 0.0
        return (v / self.pos_fullscale_v) * self.pos_fullscale_mm

    def _pres_from_v(self, v: float) -> float:
        denom = max(self.pres_vmax - self.pres_vmin, 1e-9)
        ratio = min(1.0, max(0.0, (v - self.pres_vmin) / denom))
        return ratio * (self.pres_rmax - self.pres_rmin) + self.pres_rmin

    def _forceN_from_v_L(self, v: float) -> float:
        return self.load_gain_L * (v - self.load_zero_L)

    def _forceN_from_v_R(self, v: float) -> float:
        return self.load_gain_R * (v - self.load_zero_R)

    # -------- Callback --------
    def _cb_voltage(self, msg: Float32MultiArray):
        raw = list(msg.data)
        if len(raw) < 6: return

        try:
            # 基本の変換
            L_pos_mm = self._pos_from_v(float(raw[self.index_map[0]]))
            R_pos_mm = self._pos_from_v(float(raw[self.index_map[1]]))
            L_pres_kpa = self._pres_from_v(float(raw[self.index_map[2]]))
            R_pres_kpa = self._pres_from_v(float(raw[self.index_map[3]]))
            L_force_N = self._forceN_from_v_L(float(raw[self.index_map[4]]))
            R_force_N = self._forceN_from_v_R(float(raw[self.index_map[5]]))

            # 各トピックへ発行
            self.pub_L_pos.publish(Float32(data=L_pos_mm))
            self.pub_R_pos.publish(Float32(data=R_pos_mm))
            self.pub_L_pres.publish(Float32(data=L_pres_kpa))
            self.pub_R_pres.publish(Float32(data=R_pres_kpa))
            self.pub_L_force_N.publish(Float32(data=L_force_N))
            self.pub_R_force_N.publish(Float32(data=R_force_N))

            # --- ★ 追加: インデックス11（CH11）から供給圧を取得して発行 ---
            if len(raw) > 11:
                sup_v = float(raw[11])
                sup_kpa = self._pres_from_v(sup_v)
                self.pub_sup_pres.publish(Float32(data=sup_kpa))

        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Error processing voltage data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AnalogVoltageInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()