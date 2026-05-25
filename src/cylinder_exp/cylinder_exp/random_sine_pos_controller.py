import math
import time

import numpy as np
import rclpy
from std_msgs.msg import Float32, Int32

from cylinder_exp.pos_controller import ControllerState, CylinderPositionController


class RandomSinePositionController(CylinderPositionController):
    """Cylinder position controller with randomized sine trajectory.

    The pressure/force control logic is inherited from CylinderPositionController.
    The RUNNING target trajectory is:
      x_ref_rel(t) = A_i * (1 - cos(phi(t))) in [0, 2A_i]
      d(phi)/dt = 2*pi*f_i

    A_i and f_i are re-sampled at the start of each sine cycle, where
    x_ref_rel = 0 and target velocity is also 0. This keeps the position target
    continuous even when amplitude and frequency change.
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('startup_wait_s', 8.0)
        self.declare_parameter('startup_head_voltage_v', 0.0)
        self.declare_parameter('startup_rod_voltage_v', 8.0)

        self.declare_parameter('random_sine_frequency_min_hz', 1.0)
        self.declare_parameter('random_sine_frequency_max_hz', 3.0)
        self.declare_parameter('random_sine_amplitude_min_m', 0.005)
        self.declare_parameter('random_sine_amplitude_max_m', 0.020)
        self.declare_parameter('random_sine_seed', 5)

        seed = int(self.get_parameter('random_sine_seed').value)
        self._rng = np.random.default_rng(seed)
        self._phase_rad = 0.0
        self._cycle_index = 0
        self._current_freq_hz = 0.0
        self._current_amp_m = 0.0

        self.pub_random_sine_freq = self.create_publisher(
            Float32, '/debug/random_sine_frequency_hz', 10)
        self.pub_random_sine_amp = self.create_publisher(
            Float32, '/debug/random_sine_amplitude_m', 10)
        self.pub_random_sine_phase = self.create_publisher(
            Float32, '/debug/random_sine_phase_rad', 10)
        self.pub_random_sine_cycle = self.create_publisher(
            Int32, '/debug/random_sine_cycle_index', 10)

        self._sample_next_cycle()

        self.get_logger().info(
            'RandomSinePositionController enabled. '
            f"freq=[{self.get_parameter('random_sine_frequency_min_hz').value}, "
            f"{self.get_parameter('random_sine_frequency_max_hz').value}] Hz, "
            f"amp=[{self.get_parameter('random_sine_amplitude_min_m').value}, "
            f"{self.get_parameter('random_sine_amplitude_max_m').value}] m, "
            f'seed={seed}'
        )

    def _sorted_range(self, min_param, max_param, lo=0.0):
        v_min = max(lo, float(self.get_parameter(min_param).value))
        v_max = max(lo, float(self.get_parameter(max_param).value))
        if v_min > v_max:
            v_min, v_max = v_max, v_min
        return v_min, v_max

    def _sample_next_cycle(self):
        freq_min, freq_max = self._sorted_range(
            'random_sine_frequency_min_hz',
            'random_sine_frequency_max_hz',
            lo=1e-6,
        )
        amp_min, amp_max = self._sorted_range(
            'random_sine_amplitude_min_m',
            'random_sine_amplitude_max_m',
            lo=0.0,
        )

        self._current_freq_hz = float(self._rng.uniform(freq_min, freq_max))
        self._current_amp_m = float(self._rng.uniform(amp_min, amp_max))
        self.current_sine_amp = self._current_amp_m

    def _state_waiting_sensor(self, now):
        startup_head_v = float(self.get_parameter('startup_head_voltage_v').value)
        startup_rod_v = float(self.get_parameter('startup_rod_voltage_v').value)
        self._send_valve(startup_head_v, startup_rod_v)

        if self.current_pos is not None:
            self.get_logger().info(
                f"Sensors connected. Startup wait: head={startup_head_v:.3f}V, "
                f"rod={startup_rod_v:.3f}V"
            )
            self.state = ControllerState.HOMING
            self.homing_start_time = now
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self, now):
        startup_head_v = float(self.get_parameter('startup_head_voltage_v').value)
        startup_rod_v = float(self.get_parameter('startup_rod_voltage_v').value)
        startup_wait_s = max(0.0, float(self.get_parameter('startup_wait_s').value))

        self._send_valve(startup_head_v, startup_rod_v)

        if self.homing_start_time is None:
            self.homing_start_time = now

        if now - self.homing_start_time < startup_wait_s:
            self.homing_last_pos = self.current_pos
            return

        self.x_0 = self.current_pos
        self.get_logger().info(
            f"Startup wait complete. x_0 = {self.x_0:.4f} m. "
            f"Starting random sine control."
        )

        self.pid_pos.reset()
        self.pid_pH.reset()
        self.pid_pR.reset()
        self._target_force_N = 0.0
        self._current_gain_ratio = float(
            self.get_parameter('gain_ramp_start_ratio').value
        )
        self._outer_last_time = time.monotonic()
        self.run_start_time = time.monotonic()
        self._phase_rad = 0.0
        self._cycle_index = 0
        self._sample_next_cycle()

        self.state = ControllerState.RUNNING

    def _state_running(self, now, dt):
        self._phase_rad += 2.0 * math.pi * self._current_freq_hz * dt
        while self._phase_rad >= 2.0 * math.pi:
            self._phase_rad -= 2.0 * math.pi
            self._cycle_index += 1
            self._sample_next_cycle()

        x_ref_rel = self._current_amp_m * (1.0 - math.cos(self._phase_rad))
        x_rel = self._get_relative_pos()

        pos_force = self.pid_pos.update(x_ref_rel, x_rel, dt)
        self._target_force_N = self._compose_target_force(
            pos_force, now, allow_loadcell=True
        )

        self._publish_debug(x_ref_rel, x_rel)
        self.pub_random_sine_freq.publish(Float32(data=float(self._current_freq_hz)))
        self.pub_random_sine_amp.publish(Float32(data=float(self._current_amp_m)))
        self.pub_random_sine_phase.publish(Float32(data=float(self._phase_rad)))
        self.pub_random_sine_cycle.publish(Int32(data=int(self._cycle_index)))


def main(args=None):
    rclpy.init(args=args)
    node = RandomSinePositionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node._send_all_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
