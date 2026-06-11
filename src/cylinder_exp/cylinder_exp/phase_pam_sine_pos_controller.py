#!/usr/bin/env python3
import math
import random
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from cylinder_exp.pos_controller import PIDController


class ControllerState(Enum):
    WAITING_SENSOR = auto()
    STABILIZING = auto()
    RUNNING = auto()


class PhasePamSinePositionController(Node):
    """
    Sinusoidal cylinder position controller with phase-synchronized PAM steps.

    Startup:
      head=startup_head_voltage_v, rod=startup_rod_voltage_v
      PAM target=100 kPa with PI control
      x_0 is captured at the end of startup_stabilize_s

    Running:
      x_ref_rel = A * (1 - cos(phase))
      phase=0 mod 2pi is the valley at x_0
      cycles 1-3 keep PAM target at 100 kPa with PI control
      cycles 4+ step PAM target to 350 kPa during 40-50% of the cycle
    """

    VALVE_NEUTRAL = 5.0

    def __init__(self):
        super().__init__('phase_pam_sine_position_controller')

        # Cylinder hardware and startup.
        self.declare_parameter('ch_head', 3)
        self.declare_parameter('ch_rod', 2)
        self.declare_parameter('startup_head_voltage_v', 0.0)
        self.declare_parameter('startup_rod_voltage_v', 8.0)
        self.declare_parameter('startup_stabilize_s', 5.0)

        # Cylinder sine trajectory.
        self.declare_parameter('sine_amplitude_m', 0.010)
        self.declare_parameter('sine_freq_hz', 1.0)
        self.declare_parameter('sine_amp_ramp_rate_m_s', 0.010)

        # Control rates.
        self.declare_parameter('outer_rate_hz', 500.0)
        self.declare_parameter('inner_rate_hz', 1000.0)
        self.declare_parameter('pam_control_rate_hz', 1000.0)

        # Cylinder force/pressure control.
        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)
        self.declare_parameter('pos_kp', 3500.0)
        self.declare_parameter('pos_ki', 0.0)
        self.declare_parameter('pos_kd', 0.0)
        self.declare_parameter('pos_td', 1.0)
        self.declare_parameter('pos_output_limit', 1000.0)
        self.declare_parameter('pres_kp', 0.024)
        self.declare_parameter('pres_ki', 0.05)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)
        self.declare_parameter('pres_output_limit', 4.9)

        # PAM phase-synchronized pressure control.
        self.declare_parameter('pam_valve_channel', 1)
        self.declare_parameter('pam_control_topic', '/sensors/pam_pressure')
        self.declare_parameter('pam_low_pressure_kpa', 100.0)
        self.declare_parameter('pam_high_pressure_kpa', 350.0)
        self.declare_parameter('pam_high_start_ratio', 0.40)
        self.declare_parameter('pam_high_end_ratio', 0.50)
        self.declare_parameter('pam_randomize_high_window', False)
        self.declare_parameter('pam_random_seed', 0)
        self.declare_parameter('pam_warmup_cycles', 3)
        self.declare_parameter('pam_hold_kp', 0.03)
        self.declare_parameter('pam_hold_ki', 0.01)
        self.declare_parameter('pam_step_kp', 0.03)
        self.declare_parameter('pam_step_ki', 0.0)
        self.declare_parameter('pam_output_limit', 4.9)

        self.CH_HEAD = int(self.get_parameter('ch_head').value)
        self.CH_ROD = int(self.get_parameter('ch_rod').value)
        self.PAM_CH = int(self.get_parameter('pam_valve_channel').value)


        d_cyl = 0.020
        d_rod = 0.010
        self.area_head = math.pi / 4.0 * d_cyl ** 2
        self.area_rod = math.pi / 4.0 * (d_cyl ** 2 - d_rod ** 2)
        total_area = self.area_head + self.area_rod
        self.ratio_h = self.area_head / total_area
        self.ratio_r = self.area_rod / total_area

        self.state = ControllerState.WAITING_SENSOR
        self.x_0 = 0.0
        self.current_pos = None
        self.current_pH = None
        self.current_pR = None
        self.current_pam_pressure = None

        self.current_sine_amp = 0.0
        self.run_start_time = None
        self.stabilize_start_time = None
        self._pam_rng = random.Random(int(self.get_parameter('pam_random_seed').value))
        self._pam_random_windows = {}
        self._pam_last_random_cycle = -1
        self._current_pam_high_start = None
        self._current_pam_high_end = None
        self._outer_last_time = None
        self._pam_last_time = None
        self._target_force_N = 0.0
        self._last_phase_mod = 0.0
        self._last_pam_target = None
        self._last_pam_mode = None

        self.pid_pos = PIDController(output_limit=1000.0)
        self.pid_pH = PIDController(output_limit=4.9)
        self.pid_pR = PIDController(output_limit=4.9)
        self.pid_pam = PIDController(output_limit=4.9)
        self._update_cylinder_gains()

        self.pub_cylinder_valves = self.create_publisher(
            Float32MultiArray, '/actuators/cylinder_valves', 10
        )
        self.pub_pam_valve = self.create_publisher(
            Float32MultiArray, '/actuators/pam_valve', 10
        )
        self.pub_target_pos = self.create_publisher(
            Float32, '/debug/target_position_m', 10
        )
        self.pub_sine_phase = self.create_publisher(
            Float32, '/debug/sine_phase_rad', 10
        )
        self.pub_pam_target = self.create_publisher(
            Float32, '/debug/pam_target_pressure_kPa', 10
        )
        self.pub_pam_control_pressure = self.create_publisher(
            Float32, '/debug/pam_control_pressure_kPa', 10
        )
        self.pub_pam_output = self.create_publisher(
            Float32, '/debug/pam_valve_output_V', 10
        )
        self.pub_pam_high_start = self.create_publisher(
            Float32, '/debug/pam_high_start_ratio', 10
        )
        self.pub_pam_high_end = self.create_publisher(
            Float32, '/debug/pam_high_end_ratio', 10
        )

        pam_control_topic = self.get_parameter('pam_control_topic').value
        self.create_subscription(
            Float32, '/sensors/cylinder_position', self._cb_pos, 10
        )
        self.create_subscription(Float32, '/sensors/head_pressure', self._cb_ph, 10)
        self.create_subscription(Float32, '/sensors/rod_pressure', self._cb_pr, 10)
        self.create_subscription(
            Float32, pam_control_topic, self._cb_pam_pressure, 10
        )

        outer_rate = float(self.get_parameter('outer_rate_hz').value)
        inner_rate = float(self.get_parameter('inner_rate_hz').value)
        pam_rate = float(self.get_parameter('pam_control_rate_hz').value)
        self.create_timer(1.0 / outer_rate, self._outer_loop)
        self.create_timer(1.0 / inner_rate, self._inner_loop)
        self.create_timer(1.0 / pam_rate, self._pam_loop)

        self.get_logger().info(
            'Phase PAM sine controller started. '
            f'freq={self.get_parameter("sine_freq_hz").value:.3f} Hz, '
            f'stabilize={self.get_parameter("startup_stabilize_s").value:.1f} s, '
            f'PAM high ratio='
            f'{self.get_parameter("pam_high_start_ratio").value:.2f}-'
            f'{self.get_parameter("pam_high_end_ratio").value:.2f}, '
            f'PAM randomize={self.get_parameter("pam_randomize_high_window").value}, '
            f'PAM random seed={self.get_parameter("pam_random_seed").value}, '
            f'PAM topic={pam_control_topic}'
        )

    def _cb_pos(self, msg: Float32):
        self.current_pos = float(msg.data)

    def _cb_ph(self, msg: Float32):
        self.current_pH = float(msg.data)

    def _cb_pr(self, msg: Float32):
        self.current_pR = float(msg.data)

    def _cb_pam_pressure(self, msg: Float32):
        self.current_pam_pressure = float(msg.data)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def _update_cylinder_gains(self):
        self.pid_pos.kp = float(self.get_parameter('pos_kp').value)
        self.pid_pos.ki = float(self.get_parameter('pos_ki').value)
        self.pid_pos.kd = float(self.get_parameter('pos_kd').value)
        self.pid_pos.td = float(self.get_parameter('pos_td').value)
        self.pid_pos.output_limit = float(
            self.get_parameter('pos_output_limit').value
        )

        pres_limit = float(self.get_parameter('pres_output_limit').value)
        for pid in (self.pid_pH, self.pid_pR):
            pid.kp = float(self.get_parameter('pres_kp').value)
            pid.ki = float(self.get_parameter('pres_ki').value)
            pid.kd = float(self.get_parameter('pres_kd').value)
            pid.td = float(self.get_parameter('pres_td').value)
            pid.output_limit = pres_limit

    def _set_pam_gains(self, mode):
        if mode == 'hold_pi':
            self.pid_pam.kp = float(self.get_parameter('pam_hold_kp').value)
            self.pid_pam.ki = float(self.get_parameter('pam_hold_ki').value)
        else:
            self.pid_pam.kp = float(self.get_parameter('pam_step_kp').value)
            self.pid_pam.ki = float(self.get_parameter('pam_step_ki').value)
        self.pid_pam.kd = 0.0
        self.pid_pam.td = 0.01
        self.pid_pam.output_limit = float(
            self.get_parameter('pam_output_limit').value
        )

    def _send_cylinder_valves(self, volt_h, volt_r):
        msg = Float32MultiArray()
        msg.data = [
            float(self.CH_HEAD), self._clamp(float(volt_h), 0.0, 10.0),
            float(self.CH_ROD), self._clamp(float(volt_r), 0.0, 10.0),
        ]
        self.pub_cylinder_valves.publish(msg)

    def _send_startup_cylinder_valves(self):
        self._send_cylinder_valves(
            float(self.get_parameter('startup_head_voltage_v').value),
            float(self.get_parameter('startup_rod_voltage_v').value),
        )

    def _send_neutral_outputs(self):
        self._send_cylinder_valves(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)
        self._send_pam_valve(self.VALVE_NEUTRAL)

    def _send_pam_valve(self, valve_voltage):
        msg = Float32MultiArray()
        msg.data = [
            float(self.PAM_CH),
            self._clamp(float(valve_voltage), 0.0, 10.0),
        ]
        self.pub_pam_valve.publish(msg)

    def _sensors_ready(self):
        return (
            self.current_pos is not None
            and self.current_pH is not None
            and self.current_pR is not None
            and self.current_pam_pressure is not None
        )

    def _outer_loop(self):
        now = time.monotonic()

        if self.state == ControllerState.WAITING_SENSOR:
            self._send_startup_cylinder_valves()
            if self._sensors_ready():
                self.state = ControllerState.STABILIZING
                self.stabilize_start_time = now
                self.get_logger().info(
                    'Sensors connected. Starting stabilization: '
                    f'head={self.get_parameter("startup_head_voltage_v").value:.3f} V, '
                    f'rod={self.get_parameter("startup_rod_voltage_v").value:.3f} V, '
                    f'PAM={self.get_parameter("pam_low_pressure_kpa").value:.1f} kPa'
                )
            return

        if self.state == ControllerState.STABILIZING:
            self._send_startup_cylinder_valves()
            stabilize_s = float(self.get_parameter('startup_stabilize_s').value)
            if now - self.stabilize_start_time >= stabilize_s:
                self.x_0 = float(self.current_pos)
                self.current_sine_amp = 0.0
                self.run_start_time = now
                self._reset_pam_random_windows()
                self._outer_last_time = now
                self._target_force_N = 0.0
                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self.get_logger().info(
                    f'Stabilization complete. x_0={self.x_0:.4f} m. '
                    'Starting sine control at phase=0 rad.'
                )
                self.state = ControllerState.RUNNING
            return

        if self.state != ControllerState.RUNNING:
            return

        if self._outer_last_time is None:
            self._outer_last_time = now
            return

        dt = now - self._outer_last_time
        self._outer_last_time = now
        if dt <= 0.0:
            return

        self._update_cylinder_gains()
        self._update_sine_target(now, dt)

    def _update_sine_target(self, now, dt):
        target_amp = max(0.0, float(self.get_parameter('sine_amplitude_m').value))
        amp_rate = max(
            0.0, float(self.get_parameter('sine_amp_ramp_rate_m_s').value)
        )
        delta_amp = amp_rate * dt
        if self.current_sine_amp < target_amp:
            self.current_sine_amp = min(
                self.current_sine_amp + delta_amp, target_amp
            )
        else:
            self.current_sine_amp = max(
                self.current_sine_amp - delta_amp, target_amp
            )

        freq = float(self.get_parameter('sine_freq_hz').value)
        elapsed = now - self.run_start_time
        total_phase = 2.0 * math.pi * freq * elapsed
        phase_mod = total_phase % (2.0 * math.pi)
        self._last_phase_mod = phase_mod

        x_ref_rel = self.current_sine_amp * (1.0 - math.cos(phase_mod))
        x_rel = float(self.current_pos) - self.x_0
        self._target_force_N = self.pid_pos.update(x_ref_rel, x_rel, dt)

        self.pub_target_pos.publish(Float32(data=float(self.x_0 + x_ref_rel)))
        self.pub_sine_phase.publish(Float32(data=float(phase_mod)))

    def _inner_loop(self):
        if self.state != ControllerState.RUNNING:
            return
        if self.current_pH is None or self.current_pR is None:
            return

        inner_rate = float(self.get_parameter('inner_rate_hz').value)
        self._apply_force_to_cylinder_valves(self._target_force_N, 1.0 / inner_rate)

    def _apply_force_to_cylinder_valves(self, target_force_N, dt):
        base_kpa = float(self.get_parameter('base_pressure_kpa').value)
        max_kpa = float(self.get_parameter('supply_pressure_kpa').value)

        delta_pH = (target_force_N * self.ratio_r) / self.area_head / 1000.0
        delta_pR = -(target_force_N * self.ratio_h) / self.area_rod / 1000.0
        base_rod_kpa = base_kpa * self.area_head / self.area_rod

        target_pH = self._clamp(base_kpa + delta_pH, 0.0, max_kpa)
        target_pR = self._clamp(base_rod_kpa + delta_pR, 0.0, max_kpa)

        uH = self.pid_pH.update(target_pH, float(self.current_pH), dt)
        uR = self.pid_pR.update(target_pR, float(self.current_pR), dt)

        self._send_cylinder_valves(
            self.VALVE_NEUTRAL + uH,
            self.VALVE_NEUTRAL + uR,
        )

    def _pam_loop(self):
        if self.current_pam_pressure is None:
            return

        now = time.monotonic()
        if self._pam_last_time is None:
            self._pam_last_time = now
            return

        dt = now - self._pam_last_time
        self._pam_last_time = now
        if dt <= 0.0:
            return

        target_kpa, mode = self._current_pam_target_and_mode(now)
        if self._last_pam_target != target_kpa or self._last_pam_mode != mode:
            self.pid_pam.reset()
            self._last_pam_target = target_kpa
            self._last_pam_mode = mode

        self._set_pam_gains(mode)
        u = self.pid_pam.update(
            target_kpa,
            float(self.current_pam_pressure),
            dt,
        )
        self._send_pam_valve(self.VALVE_NEUTRAL + u)

        self.pub_pam_target.publish(Float32(data=float(target_kpa)))
        self.pub_pam_control_pressure.publish(
            Float32(data=float(self.current_pam_pressure))
        )
        self.pub_pam_output.publish(Float32(data=float(u)))
        if self._current_pam_high_start is not None:
            self.pub_pam_high_start.publish(
                Float32(data=float(self._current_pam_high_start))
            )
        if self._current_pam_high_end is not None:
            self.pub_pam_high_end.publish(
                Float32(data=float(self._current_pam_high_end))
            )

    def _current_pam_target_and_mode(self, now):
        low_kpa = float(self.get_parameter('pam_low_pressure_kpa').value)
        high_kpa = float(self.get_parameter('pam_high_pressure_kpa').value)

        if self.state != ControllerState.RUNNING or self.run_start_time is None:
            return low_kpa, 'hold_pi'

        freq = float(self.get_parameter('sine_freq_hz').value)
        elapsed = max(0.0, now - self.run_start_time)
        total_phase = 2.0 * math.pi * freq * elapsed
        cycle_index = int(total_phase / (2.0 * math.pi))
        warmup_cycles = int(self.get_parameter('pam_warmup_cycles').value)

        if cycle_index < warmup_cycles:
            self._set_current_pam_high_window(cycle_index)
            return low_kpa, 'hold_pi'

        phase_mod = total_phase % (2.0 * math.pi)
        cycle_phase_ratio = phase_mod / (2.0 * math.pi)
        high_start, high_end = self._set_current_pam_high_window(cycle_index)

        if high_start <= cycle_phase_ratio <= high_end:
            return high_kpa, 'step_p'

        return low_kpa, 'step_p'

    def _reset_pam_random_windows(self):
        seed = int(self.get_parameter('pam_random_seed').value)
        self._pam_rng = random.Random(seed)
        self._pam_random_windows = {}
        self._pam_last_random_cycle = -1
        self._current_pam_high_start = None
        self._current_pam_high_end = None

    def _set_current_pam_high_window(self, cycle_index):
        high_start, high_end = self._pam_high_window_for_cycle(cycle_index)
        self._current_pam_high_start = high_start
        self._current_pam_high_end = high_end
        return high_start, high_end

    def _pam_high_window_for_cycle(self, cycle_index):
        configured_start = self._clamp(
            float(self.get_parameter('pam_high_start_ratio').value), 0.0, 1.0
        )
        configured_end = self._clamp(
            float(self.get_parameter('pam_high_end_ratio').value), 0.0, 1.0
        )
        window_ratio = max(0.0, configured_end - configured_start)
        if window_ratio <= 0.0:
            return configured_start, configured_start

        randomize = bool(self.get_parameter('pam_randomize_high_window').value)
        if not randomize:
            return configured_start, configured_end

        while self._pam_last_random_cycle < cycle_index:
            self._pam_last_random_cycle += 1
            high_start = self._pam_rng.uniform(0.0, 1.0 - window_ratio)
            self._pam_random_windows[self._pam_last_random_cycle] = (
                high_start,
                high_start + window_ratio,
            )
        return self._pam_random_windows[cycle_index]


def main(args=None):
    rclpy.init(args=args)
    node = PhasePamSinePositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node._send_neutral_outputs()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
