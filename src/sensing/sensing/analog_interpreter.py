#!/usr/bin/env python3
import collections
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class AnalogInterpreterNode(Node):
    """
    入力:
      /ai1616llpe/voltage : Float32MultiArray [V] (16ch)

    出力:
      /sensors/pressure         : Float32 [kPa]  ch7 圧力センサ (フィルタなし)
      /sensors/loadcell_force   : Float32 [N]     ロードセル (フィルタなし)
      /sensors/wire_position_mm : Float32 [mm]    ch6 ワイヤ式長さセンサ (フィルタなし)
      /RMS/deltoid              : Float32         ch1 三角筋 EMG RMS (500ms窓)
      /RMS/triceps              : Float32         ch2 上腕三頭筋 EMG RMS (500ms窓)
    """

    def __init__(self):
        super().__init__('analog_interpreter')

        # AI入力
        self.declare_parameter('ai_topic', '/ai1616llpe/voltage')

        # 圧力センサ (ch7)
        self.declare_parameter('pressure_index', 7)
        self.declare_parameter('v0_pressure', 1.0)
        self.declare_parameter('slope_kPa_per_V', 250.0)

        # ロードセル
        self.declare_parameter('loadcell_plus_index', 4)
        self.declare_parameter('loadcell_minus_index', 5)
        self.declare_parameter('v0_loadcell', 0.0)
        self.declare_parameter('kg_per_V_loadcell', 7.9186)
        self.declare_parameter('gravity_acceleration', 9.80665)

        # ワイヤ式長さセンサ (ch6): v / pos_fullscale_v * pos_fullscale_mm
        self.declare_parameter('wire_index', 6)
        self.declare_parameter('pos_fullscale_v', 5.0)
        self.declare_parameter('pos_fullscale_mm', 1000.0)

        # EMG (ch1: 三角筋, ch2: 上腕三頭筋)
        self.declare_parameter('emg_deltoid_index', 1)
        self.declare_parameter('emg_triceps_index', 2)
        self.declare_parameter('emg_rms_window_ms', 500)

        ai_topic        = self.get_parameter('ai_topic').value

        self._pi        = int(self.get_parameter('pressure_index').value)
        self._v0p       = float(self.get_parameter('v0_pressure').value)
        self._slope_kpa = float(self.get_parameter('slope_kPa_per_V').value)

        self._lc_plus   = int(self.get_parameter('loadcell_plus_index').value)
        self._lc_minus  = int(self.get_parameter('loadcell_minus_index').value)
        self._lc_v0     = float(self.get_parameter('v0_loadcell').value)
        self._lc_kgpv   = float(self.get_parameter('kg_per_V_loadcell').value)
        self._gravity   = float(self.get_parameter('gravity_acceleration').value)

        self._wi        = int(self.get_parameter('wire_index').value)
        self._pos_fs_v  = float(self.get_parameter('pos_fullscale_v').value)
        self._pos_fs_mm = float(self.get_parameter('pos_fullscale_mm').value)

        self._emg_d_idx = int(self.get_parameter('emg_deltoid_index').value)
        self._emg_t_idx = int(self.get_parameter('emg_triceps_index').value)
        rms_window_ms   = int(self.get_parameter('emg_rms_window_ms').value)
        self._rms_window_ns = rms_window_ms * 1_000_000

        # EMGバッファ: deque of (timestamp_ns, value)
        self._buf_deltoid: collections.deque = collections.deque()
        self._buf_triceps: collections.deque = collections.deque()

        self.pub_pressure = self.create_publisher(Float32, '/sensors/pressure', 10)
        self.pub_loadcell = self.create_publisher(Float32, '/sensors/loadcell_force', 10)
        self.pub_wire     = self.create_publisher(Float32, '/sensors/wire_position_mm', 10)
        self.pub_deltoid  = self.create_publisher(Float32, '/RMS/deltoid', 10)
        self.pub_triceps  = self.create_publisher(Float32, '/RMS/triceps', 10)

        self.create_subscription(Float32MultiArray, ai_topic, self._cb_voltage, 10)

        self.get_logger().info(
            f'AnalogInterpreter started. '
            f'pressure=ch{self._pi}, wire=ch{self._wi}, '
            f'loadcell=ch{self._lc_plus}-ch{self._lc_minus}, '
            f'EMG deltoid=ch{self._emg_d_idx}, triceps=ch{self._emg_t_idx}, '
            f'RMS window={rms_window_ms}ms'
        )

    def _cb_voltage(self, msg: Float32MultiArray):
        arr = msg.data
        now_ns = self.get_clock().now().nanoseconds

        # 圧力センサ: (V - v0) * slope
        if self._pi < len(arr):
            kpa = (float(arr[self._pi]) - self._v0p) * self._slope_kpa
            self.pub_pressure.publish(Float32(data=kpa))

        # ロードセル: ((V+ - V-) - v0) * kg/V * g
        if self._lc_plus < len(arr) and self._lc_minus < len(arr):
            v_net = (float(arr[self._lc_plus]) - float(arr[self._lc_minus])) - self._lc_v0
            force_n = v_net * self._lc_kgpv * self._gravity
            self.pub_loadcell.publish(Float32(data=force_n))

        # ワイヤ式長さセンサ: v / pos_fullscale_v * pos_fullscale_mm
        if self._wi < len(arr) and self._pos_fs_v > 1e-9:
            mm = float(arr[self._wi]) / self._pos_fs_v * self._pos_fs_mm
            self.pub_wire.publish(Float32(data=mm))

        # EMG RMS
        if self._emg_d_idx < len(arr):
            self._update_rms(self._buf_deltoid, float(arr[self._emg_d_idx]), now_ns,
                             self.pub_deltoid)
        if self._emg_t_idx < len(arr):
            self._update_rms(self._buf_triceps, float(arr[self._emg_t_idx]), now_ns,
                             self.pub_triceps)

    def _update_rms(self, buf: collections.deque, value: float, now_ns: int, publisher):
        buf.append((now_ns, value))
        cutoff_ns = now_ns - self._rms_window_ns
        while buf and buf[0][0] < cutoff_ns:
            buf.popleft()
        rms = math.sqrt(sum(v * v for _, v in buf) / len(buf))
        publisher.publish(Float32(data=rms))


def main(args=None):
    rclpy.init(args=args)
    node = AnalogInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
