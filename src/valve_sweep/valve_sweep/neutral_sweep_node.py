#!/usr/bin/env python3
"""
Two-step neutral voltage test for an MPYE servo valve.

Sequence:
  1. Apply initial_voltage to valve_channel for initial_duration_s.
  2. Apply target_voltage to valve_channel for target_duration_s.
  3. Publish neutral_default once and exit.

Outputs:
  /actuators/valve_voltage              : Float32MultiArray [8ch voltages]
  /debug/neutral_sweep_valve_voltage_V : Float32 selected valve command
  /debug/neutral_sweep_ai_ch6_raw_V    : Float32 selected AI raw voltage
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class NeutralSweepNode(Node):
    """Apply initial and target voltages while exposing scalar log topics."""

    def __init__(self):
        super().__init__('neutral_sweep_node')

        self.declare_parameter('valve_channel', 1)
        self.declare_parameter('monitor_ai_channel', 6)
        self.declare_parameter('initial_voltage', 5.8)
        self.declare_parameter('target_voltage', 5.0)
        self.declare_parameter('initial_duration_s', 3.0)
        self.declare_parameter('target_duration_s', 3.0)
        self.declare_parameter('sample_rate_hz', 100.0)
        self.declare_parameter('neutral_default', 5.0)

        self.valve_ch = int(self.get_parameter('valve_channel').value)
        self.monitor_ai_ch = int(self.get_parameter('monitor_ai_channel').value)
        self.initial_voltage = float(self.get_parameter('initial_voltage').value)
        self.target_voltage = float(self.get_parameter('target_voltage').value)
        self.initial_duration = float(
            self.get_parameter('initial_duration_s').value
        )
        self.target_duration = float(
            self.get_parameter('target_duration_s').value
        )
        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.neutral_default = float(self.get_parameter('neutral_default').value)

        self._validate_parameters()

        self._start_time = self.get_clock().now()
        self._finished = False
        self._ai_voltage = None
        self._shutdown_timer = None

        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10
        )
        self.pub_debug_valve = self.create_publisher(
            Float32, '/debug/neutral_sweep_valve_voltage_V', 10
        )
        self.pub_debug_ai = self.create_publisher(
            Float32, '/debug/neutral_sweep_ai_ch6_raw_V', 10
        )

        self.create_subscription(
            Float32MultiArray,
            '/ai1616llpe/voltage',
            self._cb_ai_voltage,
            10,
        )

        self.create_timer(1.0 / self.sample_rate_hz, self._control_loop)

        self.get_logger().info(
            'Neutral sweep started: '
            f'ch={self.valve_ch}, initial={self.initial_voltage:.3f} V '
            f'for {self.initial_duration:.3f} s, '
            f'target={self.target_voltage:.3f} V '
            f'for {self.target_duration:.3f} s, '
            f'rate={self.sample_rate_hz:.1f} Hz, '
            f'AI monitor ch={self.monitor_ai_ch}'
        )

    def _validate_parameters(self):
        if not 0 <= self.valve_ch < 8:
            raise ValueError('valve_channel must be in [0, 7]')
        if not 0 <= self.monitor_ai_ch < 16:
            raise ValueError('monitor_ai_channel must be in [0, 15]')
        if self.sample_rate_hz <= 0.0:
            raise ValueError('sample_rate_hz must be positive')
        if self.initial_duration < 0.0 or self.target_duration < 0.0:
            raise ValueError('durations must be non-negative')

        for name, voltage in (
            ('initial_voltage', self.initial_voltage),
            ('target_voltage', self.target_voltage),
            ('neutral_default', self.neutral_default),
        ):
            if not math.isfinite(voltage) or not 0.0 <= voltage <= 10.0:
                raise ValueError(f'{name} must be finite and in [0.0, 10.0] V')

    def _cb_ai_voltage(self, msg: Float32MultiArray):
        if self.monitor_ai_ch < len(msg.data):
            self._ai_voltage = float(msg.data[self.monitor_ai_ch])

    def _control_loop(self):
        if self._finished:
            return

        elapsed = (
            self.get_clock().now() - self._start_time
        ).nanoseconds / 1e9

        if elapsed < self.initial_duration:
            voltage = self.initial_voltage
        elif elapsed < self.initial_duration + self.target_duration:
            voltage = self.target_voltage
        else:
            self._finish()
            return

        self._publish_valve(voltage)
        self.pub_debug_valve.publish(Float32(data=voltage))
        if self._ai_voltage is not None:
            self.pub_debug_ai.publish(Float32(data=self._ai_voltage))

    def _publish_valve(self, selected_voltage: float):
        msg = Float32MultiArray()
        voltages = [self.neutral_default] * 8
        voltages[self.valve_ch] = selected_voltage
        msg.data = voltages
        self.pub_valve.publish(msg)

    def _finish(self):
        self._finished = True
        self._publish_valve(self.neutral_default)
        self.pub_debug_valve.publish(Float32(data=self.neutral_default))
        self.get_logger().info(
            'Neutral sweep completed. Published neutral and exiting.'
        )
        self._shutdown_timer = self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self):
        self._publish_valve(self.neutral_default)
        self.pub_debug_valve.publish(Float32(data=self.neutral_default))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NeutralSweepNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutting down...')
    finally:
        if rclpy.ok():
            node._publish_valve(node.neutral_default)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
