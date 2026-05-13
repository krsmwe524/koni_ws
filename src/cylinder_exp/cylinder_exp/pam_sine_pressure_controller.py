import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from cylinder_exp.pam_const_pressure_controller import PIDController


class PamSinePressureController(Node):
    """
    PAM pressure controller with a sinusoidal pressure target.

    Target pressure:
      before sine_start_delay_s:
        target = base_pressure_kpa
      after sine_start_delay_s:
        target = base_pressure_kpa
                 + amplitude_kpa * sin(2*pi*sine_freq_hz*t)

    Input:
      control_topic : Float32 [kPa]

    Output:
      valve_topic : Float32MultiArray [ch, volt]
    """

    VALVE_NEUTRAL = 5.0

    def __init__(self):
        super().__init__('pam_sine_pressure_controller')

        self.declare_parameter('base_pressure_kpa', 100.0)
        self.declare_parameter('amplitude_kpa', 5.0)
        self.declare_parameter('sine_freq_hz', 0.5)
        self.declare_parameter('sine_start_delay_s', 8.0)

        self.declare_parameter('kp', 0.02)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('td', 0.01)
        self.declare_parameter('derivative_enable_delay_s', 0.0)
        self.declare_parameter('output_limit', 4.9)
        self.declare_parameter('valve_channel', 3)
        self.declare_parameter('control_rate_hz', 500.0)
        self.declare_parameter('control_topic', '/sensors/pam_pressure')
        self.declare_parameter('valve_topic', '/actuators/pam_valve')

        kp = float(self.get_parameter('kp').value)
        ki = float(self.get_parameter('ki').value)
        kd = float(self.get_parameter('kd').value)
        td = float(self.get_parameter('td').value)
        output_limit = float(self.get_parameter('output_limit').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.derivative_enable_delay_s = float(
            self.get_parameter('derivative_enable_delay_s').value)
        self.channel = int(self.get_parameter('valve_channel').value)
        self.control_topic = self.get_parameter('control_topic').value
        valve_topic = self.get_parameter('valve_topic').value

        self.pid = PIDController(
            kp=kp, ki=ki, kd=kd, td=td, output_limit=output_limit)

        self.current_pressure = None
        self._last_time = None
        self._start_time = time.monotonic()

        self.pub_valve = self.create_publisher(
            Float32MultiArray, valve_topic, 10)
        self.pub_debug_target = self.create_publisher(
            Float32, '/debug/pam_target_pressure_kPa', 10)
        self.pub_debug_control_pressure = self.create_publisher(
            Float32, '/debug/pam_control_pressure_kPa', 10)
        self.pub_debug_control_error = self.create_publisher(
            Float32, '/debug/pam_control_pressure_error_kPa', 10)
        self.pub_debug_error = self.create_publisher(
            Float32, '/debug/pam_pressure_error_kPa', 10)
        self.pub_debug_output = self.create_publisher(
            Float32, '/debug/pam_valve_output_V', 10)
        self.pub_debug_derivative = self.create_publisher(
            Float32, '/debug/pam_pressure_error_derivative_kPa_s', 10)

        self.pub_debug_base = self.create_publisher(
            Float32, '/debug/pam_sine_base_pressure_kPa', 10)
        self.pub_debug_amplitude = self.create_publisher(
            Float32, '/debug/pam_sine_amplitude_kPa', 10)
        self.pub_debug_phase = self.create_publisher(
            Float32, '/debug/pam_sine_phase_rad', 10)

        self.create_subscription(Float32, self.control_topic,
                                 self._cb_pressure, 10)
        self.create_timer(1.0 / rate_hz, self._control_loop)

        self.get_logger().info(
            f"PAM Sine Pressure Controller started. "
            f"base={self.get_parameter('base_pressure_kpa').value:.1f} kPa, "
            f"amplitude={self.get_parameter('amplitude_kpa').value:.1f} kPa, "
            f"freq={self.get_parameter('sine_freq_hz').value:.3f} Hz, "
            f"start_delay={self.get_parameter('sine_start_delay_s').value:.1f}s, "
            f"control_topic={self.control_topic}, "
            f"kp={kp}, ki={ki}, kd={kd}, td={td}, "
            f"d_enable_delay={self.derivative_enable_delay_s}s, ch={self.channel}"
        )

    def _cb_pressure(self, msg: Float32):
        self.current_pressure = float(msg.data)

    def _target_pressure(self, now):
        base_kpa = float(self.get_parameter('base_pressure_kpa').value)
        amplitude_kpa = float(self.get_parameter('amplitude_kpa').value)
        freq_hz = float(self.get_parameter('sine_freq_hz').value)
        start_delay_s = max(
            0.0, float(self.get_parameter('sine_start_delay_s').value))

        elapsed = now - self._start_time
        if elapsed < start_delay_s:
            return base_kpa, base_kpa, amplitude_kpa, 0.0

        sine_t = elapsed - start_delay_s
        phase = 2.0 * math.pi * freq_hz * sine_t
        target_kpa = base_kpa + amplitude_kpa * math.sin(phase)
        return target_kpa, base_kpa, amplitude_kpa, phase

    def _control_loop(self):
        now = time.monotonic()

        if self.current_pressure is None:
            return

        if self._last_time is None:
            self._last_time = now
            return

        dt = now - self._last_time
        self._last_time = now
        if dt <= 0.0:
            return

        target_kpa, base_kpa, amplitude_kpa, phase = self._target_pressure(now)
        derivative_enabled = (
            now - self._start_time) >= self.derivative_enable_delay_s
        u = self.pid.update(
            target_kpa,
            self.current_pressure,
            dt,
            derivative_enabled=derivative_enabled,
        )

        valve_volt = max(0.0, min(10.0, self.VALVE_NEUTRAL + u))

        msg = Float32MultiArray()
        msg.data = [float(self.channel), valve_volt]
        self.pub_valve.publish(msg)

        error = target_kpa - self.current_pressure
        self.pub_debug_target.publish(Float32(data=float(target_kpa)))
        self.pub_debug_control_pressure.publish(
            Float32(data=float(self.current_pressure)))
        self.pub_debug_control_error.publish(Float32(data=float(error)))
        self.pub_debug_error.publish(Float32(data=float(error)))
        self.pub_debug_output.publish(Float32(data=float(u)))
        self.pub_debug_derivative.publish(
            Float32(data=float(self.pid.last_derivative)))
        self.pub_debug_base.publish(Float32(data=float(base_kpa)))
        self.pub_debug_amplitude.publish(Float32(data=float(amplitude_kpa)))
        self.pub_debug_phase.publish(Float32(data=float(phase)))


def main(args=None):
    rclpy.init(args=args)
    node = PamSinePressureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        msg = Float32MultiArray()
        msg.data = [float(node.channel), PamSinePressureController.VALVE_NEUTRAL]
        node.pub_valve.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
