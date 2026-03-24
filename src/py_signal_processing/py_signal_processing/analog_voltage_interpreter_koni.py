#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32

class AnalogVoltageInterpreterNode(Node):
    """
    入力:
      /ai1616llpe/voltage : std_msgs/Float32MultiArray  [V]
        例: [L_posV, R_posV, L_presV, R_presV, L_loadV, R_loadV]
    出力:
      /sensors/left_load  : std_msgs/Float32  [N]
      /sensors/right_load : std_msgs/Float32  [N]
      (任意) /sensors/left_load_kg  /sensors/right_load_kg : kgf
    """

    def __init__(self):
        super().__init__('analog_voltage_interpreter')

        self.declare_parameter('input_topic', '/ai1616llpe/voltage')
        self.declare_parameter('left_index',  4)
        self.declare_parameter('right_index', 5) 

        #   0.110 V → 21.57463 N → 116.62 N/V
        self.declare_parameter('v0_left',  -0.075)
        self.declare_parameter('v0_right', -0.075)
        self.declare_parameter('slope_N_per_V_left',  116.62)
        self.declare_parameter('slope_N_per_V_right', 116.62)


        self.declare_parameter('publish_kgf', False) 
        self.declare_parameter('deadband_N', 0.0) 
        self.declare_parameter('clip_negative', False) 


        self.input_topic  = self.get_parameter('input_topic').value
        self.li           = int(self.get_parameter('left_index').value)
        self.ri           = int(self.get_parameter('right_index').value)
        self.v0L          = float(self.get_parameter('v0_left').value)
        self.v0R          = float(self.get_parameter('v0_right').value)
        self.kNL          = float(self.get_parameter('slope_N_per_V_left').value)
        self.kNR          = float(self.get_parameter('slope_N_per_V_right').value)
        self.publish_kgf  = bool(self.get_parameter('publish_kgf').value)
        self.deadband_N   = float(self.get_parameter('deadband_N').value)
        self.clip_negative= bool(self.get_parameter('clip_negative').value)

    
        self.pub_L_N  = self.create_publisher(Float32, '/sensors/left_load',  10) 
        self.pub_R_N  = self.create_publisher(Float32, '/sensors/right_load', 10) 
        if self.publish_kgf:
            self.pub_L_kg = self.create_publisher(Float32, '/sensors/left_load_kg', 10)
            self.pub_R_kg = self.create_publisher(Float32, '/sensors/right_load_kg', 10)


        self.sub = self.create_subscription(
            Float32MultiArray, self.input_topic, self._cb_voltage, 10
        )

        self.get_logger().info(
            f"[{self.get_name()}] input={self.input_topic} | "
            f"indices L={self.li}, R={self.ri} | "
            f"L: (V - {self.v0L:+.3f})*{self.kNL:.2f} N/V, "
            f"R: (V - {self.v0R:+.3f})*{self.kNR:.2f} N/V | "
            f"deadband={self.deadband_N} N, clip_negative={self.clip_negative}, kgf_out={self.publish_kgf}"
        )

    @staticmethod
    def _apply_deadband(x: float, db: float) -> float:
        if db <= 0.0:
            return x
        if abs(x) < db:
            return 0.0
        return x - db if x > 0 else x + db

    def _cb_voltage(self, msg: Float32MultiArray):
        arr = list(msg.data)
        # 安全チェック
        if self.li >= len(arr) or self.ri >= len(arr):
            self.get_logger().warn(f"input array too short (len={len(arr)}), li={self.li}, ri={self.ri}")
            return

        VL = float(arr[self.li])
        VR = float(arr[self.ri])

        FL_N = (VL - self.v0L) * self.kNL
        FR_N = (VR - self.v0R) * self.kNR

        FL_N = self._apply_deadband(FL_N, self.deadband_N)
        FR_N = self._apply_deadband(FR_N, self.deadband_N)
        if self.clip_negative:
            FL_N = max(0.0, FL_N)
            FR_N = max(0.0, FR_N)

        # Publish N
        self.pub_L_N.publish(Float32(data=float(FL_N)))
        self.pub_R_N.publish(Float32(data=float(FR_N)))

        # もし kgf も出すなら
        if self.publish_kgf:
            g = 9.80665
            self.pub_L_kg.publish(Float32(data=float(FL_N / g)))
            self.pub_R_kg.publish(Float32(data=float(FR_N / g)))

def main(args=None):
    rclpy.init(args=args)
    node = AnalogVoltageInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
