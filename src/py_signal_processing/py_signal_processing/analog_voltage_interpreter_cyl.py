#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from py_signal_processing.low_pass_filter import LowPassFilter

class SensorInterpreterNode(Node):
    """
    入力:
      /ai1616llpe/voltage : Float32MultiArray [V]
      /cnt3204mtlpe       : Float32MultiArray
    出力:
      /sensors/cylinder_position : Float32 [m]
      /sensors/head_pressure     : Float32 [kPa] (ヘッド側圧力)
      /sensors/rod_pressure      : Float32 [kPa] (ロッド側圧力)
      /sensors/loadcell_force    : Float32 [N]   (ロードセルの力)
      /sensors/pam_pressure        : Float32 [kPa] (PAM直前圧力)
      /sensors/pam_valve_pressure  : Float32 [kPa] (PAMバルブ直後圧力)
      /sensors/supply_pressure     : Float32 [kPa] (供給圧力)
    """
    def __init__(self):
        super().__init__('sensor_interpreter')

        # エンコーダ
        self.declare_parameter('cnt_topic', '/cnt3204mtlpe')
        self.declare_parameter('encoder_index', 0)
        self.declare_parameter('meters_per_count', 0.00001)
        self.declare_parameter('reverse_direction', False)
        self.declare_parameter('cutoff_hz_encoder', 20.0)

        # 圧力センサ
        self.declare_parameter('ai_topic', '/ai1616llpe/voltage')
        self.declare_parameter('head_pressure_index', 0)
        self.declare_parameter('rod_pressure_index', 1)
        self.declare_parameter('pam_pressure_index', 5)
        self.declare_parameter('pam_valve_pressure_index', 6)
        self.declare_parameter('supply_pressure_index', 4)
        self.declare_parameter('v0_pressure_head', 1.0)
        self.declare_parameter('v0_pressure_rod', 1.0)
        self.declare_parameter('v0_pressure_pam', 1.0)
        self.declare_parameter('v0_pressure_pam_valve', 1.0)
        self.declare_parameter('v0_pressure_supply', 1.0)
        self.declare_parameter('slope_kPa_per_V_head', 250.0)
        self.declare_parameter('slope_kPa_per_V_rod', 250.0)
        self.declare_parameter('slope_kPa_per_V_pam', 250.0)
        self.declare_parameter('slope_kPa_per_V_pam_valve', 250.0)
        self.declare_parameter('slope_kPa_per_V_supply', 250.0)
        self.declare_parameter('cutoff_hz_pressure', 20.0)

        # ロードセル
        self.declare_parameter('loadcell_plus_index', 2)
        self.declare_parameter('loadcell_minus_index', 3)
        self.declare_parameter('v0_loadcell', 0.0)
        self.declare_parameter('kg_per_V_loadcell', 7.9186)
        self.declare_parameter('gravity_acceleration', 9.80665)
        self.declare_parameter('cutoff_hz_loadcell', 5.0)

        self.cnt_topic   = self.get_parameter('cnt_topic').value
        self.ei          = self.get_parameter('encoder_index').value
        self.m_per_count = self.get_parameter('meters_per_count').value
        self.reverse_dir = self.get_parameter('reverse_direction').value
        self.cutoff_enc  = self.get_parameter('cutoff_hz_encoder').value

        self.ai_topic    = self.get_parameter('ai_topic').value
        self.hi          = self.get_parameter('head_pressure_index').value
        self.ri          = self.get_parameter('rod_pressure_index').value
        self.pi          = self.get_parameter('pam_pressure_index').value
        self.pvi         = self.get_parameter('pam_valve_pressure_index').value
        self.si          = self.get_parameter('supply_pressure_index').value
        self.v0H_press   = self.get_parameter('v0_pressure_head').value
        self.v0R_press   = self.get_parameter('v0_pressure_rod').value
        self.v0Pam_press = self.get_parameter('v0_pressure_pam').value
        self.v0PamValve_press = self.get_parameter('v0_pressure_pam_valve').value
        self.v0S_press   = self.get_parameter('v0_pressure_supply').value
        self.kH_press    = self.get_parameter('slope_kPa_per_V_head').value
        self.kR_press    = self.get_parameter('slope_kPa_per_V_rod').value
        self.kpam_press  = self.get_parameter('slope_kPa_per_V_pam').value
        self.kPamValve_press = self.get_parameter('slope_kPa_per_V_pam_valve').value
        self.kS_press    = self.get_parameter('slope_kPa_per_V_supply').value
        self.cutoff_pres = self.get_parameter('cutoff_hz_pressure').value

        self.lc_p_idx    = self.get_parameter('loadcell_plus_index').value
        self.lc_m_idx    = self.get_parameter('loadcell_minus_index').value
        self.lc_v0       = self.get_parameter('v0_loadcell').value
        self.lc_kg_per_v = self.get_parameter('kg_per_V_loadcell').value
        self.gravity     = self.get_parameter('gravity_acceleration').value
        self.cutoff_lc   = self.get_parameter('cutoff_hz_loadcell').value

        self.lpf_enc    = LowPassFilter(self.cutoff_enc)
        self.lpf_head   = LowPassFilter(self.cutoff_pres)
        self.lpf_rod    = LowPassFilter(self.cutoff_pres)
        self.lpf_pam    = LowPassFilter(self.cutoff_pres)
        self.lpf_pam_valve = LowPassFilter(self.cutoff_pres)
        self.lpf_supply = LowPassFilter(self.cutoff_pres)
        self.lpf_lc     = LowPassFilter(self.cutoff_lc)

        self.pub_pos_m      = self.create_publisher(Float32, '/sensors/cylinder_position', 10)
        self.pub_head_kpa   = self.create_publisher(Float32, '/sensors/head_pressure', 10)
        self.pub_rod_kpa    = self.create_publisher(Float32, '/sensors/rod_pressure', 10)
        self.pub_loadcell_n = self.create_publisher(Float32, '/sensors/loadcell_force', 10)
        self.pub_pam_kpa    = self.create_publisher(Float32, '/sensors/pam_pressure', 10)
        self.pub_pam_valve_kpa = self.create_publisher(Float32, '/sensors/pam_valve_pressure', 10)
        self.pub_supply_kpa = self.create_publisher(Float32, '/sensors/supply_pressure', 10)

        self.sub_cnt = self.create_subscription(Float32MultiArray, self.cnt_topic, self._cb_count, 10)
        self.sub_ai  = self.create_subscription(Float32MultiArray, self.ai_topic, self._cb_voltage, 10)

        self.get_logger().info("Sensor Interpreter Node Started with Filters & Loadcell.")

    def _cb_count(self, msg: Float32MultiArray):
        """ 位置(m)に変換 """
        arr = msg.data
        if self.ei >= len(arr):
            return

        # 1. 生データの計算
        relative_count = float(arr[self.ei])
        raw_position_m = relative_count * self.m_per_count
        if self.reverse_dir:
            raw_position_m = -raw_position_m

        # 2. フィルタの適用
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        filtered_position_m = self.lpf_enc.update(raw_position_m, current_time_sec)

        # 3. 送信
        self.pub_pos_m.publish(Float32(data=filtered_position_m))

    def _cb_voltage(self, msg: Float32MultiArray):
        """ 電圧を圧力(kPa)と力(N)に変換 """
        arr = msg.data
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        # --- 圧力センサの処理 
        if self.hi < len(arr) and self.ri < len(arr):
            # 生データ
            V_head = float(arr[self.hi])
            V_rod  = float(arr[self.ri])
            raw_P_head_kPa = (V_head - self.v0H_press) * self.kH_press
            raw_P_rod_kPa  = (V_rod  - self.v0R_press) * self.kR_press

            # フィルタ適用
            filtered_P_head = self.lpf_head.update(raw_P_head_kPa, current_time_sec)
            filtered_P_rod  = self.lpf_rod.update(raw_P_rod_kPa, current_time_sec)

            self.pub_head_kpa.publish(Float32(data=filtered_P_head))
            self.pub_rod_kpa.publish(Float32(data=filtered_P_rod))

        # PAM直前圧力センサの処理
        if self.pi < len(arr):
            V_pam = float(arr[self.pi])
            raw_P_pam_kPa = (V_pam - self.v0Pam_press) * self.kpam_press
            filtered_P_pam = self.lpf_pam.update(raw_P_pam_kPa, current_time_sec)
            self.pub_pam_kpa.publish(Float32(data=filtered_P_pam))

        # PAMバルブ直後圧力センサの処理
        if self.pvi < len(arr):
            V_pam_valve = float(arr[self.pvi])
            raw_P_pam_valve_kPa = (
                V_pam_valve - self.v0PamValve_press
            ) * self.kPamValve_press
            filtered_P_pam_valve = self.lpf_pam_valve.update(
                raw_P_pam_valve_kPa,
                current_time_sec,
            )
            self.pub_pam_valve_kpa.publish(Float32(data=filtered_P_pam_valve))

        # 供給圧力センサの処理
        if self.si < len(arr):
            V_supply = float(arr[self.si])
            raw_P_supply_kPa = (V_supply - self.v0S_press) * self.kS_press
            filtered_P_supply = self.lpf_supply.update(raw_P_supply_kPa, current_time_sec)
            self.pub_supply_kpa.publish(Float32(data=filtered_P_supply))

        # --- ロードセルの処理 ---
        if self.lc_p_idx < len(arr) and self.lc_m_idx < len(arr):
            # 生データ
            V_plus = float(arr[self.lc_p_idx])
            V_minus = float(arr[self.lc_m_idx])
            V_net = (V_plus - V_minus) - self.lc_v0
            # 符号逆バージョン
            # V_net = (V_minus - V_plus) - self.lc_v0
            raw_force_N = (V_net * self.lc_kg_per_v) * self.gravity
            
            # フィルタ適用
            filtered_force_N = self.lpf_lc.update(raw_force_N, current_time_sec)
            
            self.pub_loadcell_n.publish(Float32(data=filtered_force_N))

def main(args=None):
    rclpy.init(args=args)
    node = SensorInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()