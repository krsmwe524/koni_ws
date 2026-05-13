import math
import time

import numpy as np
import rclpy
from std_msgs.msg import Float32, Int32

from cylinder_exp.pos_controller import ControllerState, CylinderPositionController


class RandomWaypointPositionController(CylinderPositionController):
    """Cylinder position controller with smooth random target positions.

    The pressure/force control logic is inherited from CylinderPositionController.
    Only the RUNNING target trajectory is replaced:
      r_i ~ Uniform(-1, 1) at each waypoint
      r(t) is cosine-interpolated between waypoints
      x_ref_rel(t) = A(t) * (1 + r(t)) in [0, 2A]
    """

    def __init__(self):
        super().__init__()

        self.declare_parameter('startup_wait_s', 8.0)
        self.declare_parameter('startup_head_voltage_v', 0.0)
        self.declare_parameter('startup_rod_voltage_v', 8.0)

        self.declare_parameter('random_amplitude_m', 0.020)
        self.declare_parameter('random_amp_ramp_rate_m_s', 0.0005)
        self.declare_parameter('waypoint_period_s', 0.5)
        self.declare_parameter('waypoint_seed', 1)

        seed = int(self.get_parameter('waypoint_seed').value)
        self._rng = np.random.default_rng(seed)
        self._random_amp = 0.0
        self._wp_start_time = None
        self._wp_index = 0
        self._wp_prev = -1.0
        self._wp_next = None
        self._last_random_value = -1.0

        self.pub_random_value = self.create_publisher(
            Float32, '/debug/random_waypoint_value', 10)
        self.pub_random_index = self.create_publisher(
            Int32, '/debug/random_waypoint_index', 10)
        self.pub_random_amp = self.create_publisher(
            Float32, '/debug/current_random_amplitude_m', 10)

        self.get_logger().info(
            'RandomWaypointPositionController enabled. '
            f"A={self.get_parameter('random_amplitude_m').value} m, "
            f"period={self.get_parameter('waypoint_period_s').value} s, "
            f'seed={seed}'
        )

    def _reset_random_trajectory(self, now):
        self._random_amp = 0.0
        self._wp_start_time = now
        self._wp_index = 0
        self._wp_prev = -1.0
        self._wp_next = float(self._rng.uniform(-1.0, 1.0))
        self._last_random_value = self._wp_prev

    def _advance_waypoint_if_needed(self, now, period_s):
        if self._wp_start_time is None:
            self._reset_random_trajectory(now)

        while now - self._wp_start_time >= period_s:
            self._wp_start_time += period_s
            self._wp_prev = self._wp_next
            self._wp_next = float(self._rng.uniform(-1.0, 1.0))
            self._wp_index += 1

    def _smooth_random_value(self, now, period_s):
        self._advance_waypoint_if_needed(now, period_s)
        phase = (now - self._wp_start_time) / period_s
        phase = max(0.0, min(1.0, phase))
        s = 0.5 - 0.5 * math.cos(math.pi * phase)
        return (1.0 - s) * self._wp_prev + s * self._wp_next

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
            f"Starting random waypoint control."
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
        self.current_sine_amp = 0.0
        self._reset_random_trajectory(self.run_start_time)

        self.state = ControllerState.RUNNING

    def _state_running(self, now, dt):
        target_amp = max(0.0, float(self.get_parameter('random_amplitude_m').value))
        amp_rate = max(0.0, float(self.get_parameter('random_amp_ramp_rate_m_s').value))
        period_s = max(1e-3, float(self.get_parameter('waypoint_period_s').value))

        delta_amp = amp_rate * dt
        if self._random_amp < target_amp:
            self._random_amp = min(self._random_amp + delta_amp, target_amp)
        else:
            self._random_amp = max(self._random_amp - delta_amp, target_amp)

        r = self._smooth_random_value(now, period_s)
        self._last_random_value = r

        # x_0 is captured at the end of startup_wait_s.
        # Keep the target inside [x_0, x_0 + 2A].
        x_ref_rel = self._random_amp * (1.0 + r)
        x_rel = self._get_relative_pos()

        pos_force = self.pid_pos.update(x_ref_rel, x_rel, dt)
        self._target_force_N = self._compose_target_force(
            pos_force, now, allow_loadcell=True
        )

        self._publish_debug(x_ref_rel, x_rel)
        self.pub_random_value.publish(Float32(data=float(r)))
        self.pub_random_index.publish(Int32(data=int(self._wp_index)))
        self.pub_random_amp.publish(Float32(data=float(self._random_amp)))


def main(args=None):
    rclpy.init(args=args)
    node = RandomWaypointPositionController()

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
