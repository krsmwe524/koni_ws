import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math
import time
from enum import Enum, auto


class ControllerState(Enum):
    WAITING_SENSOR = auto()
    HOMING = auto()
    PRESSURIZE = auto()
    RUNNING = auto()
    STOPPED = auto()


class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, td=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.td = td

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0
        self.output_limit = 1000.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def update(self, target, actual, dt):
        if dt <= 0.0:
            return 0.0

        error = target - actual

        # 積分 + アンチワインドアップ
        self.integral += error * dt
        if self.ki > 1e-9:
            integral_limit = self.output_limit / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

        # 不完全微分
        raw_derivative = (error - self.prev_error) / dt
        alpha = self.td / (self.td + dt)
        filtered_derivative = alpha * self.prev_derivative + (1.0 - alpha) * raw_derivative

        output = self.kp * error + self.ki * self.integral + self.kd * filtered_derivative

        self.prev_error = error
        self.prev_derivative = filtered_derivative

        return max(-self.output_limit, min(self.output_limit, output))


class CylinderPositionController(Node):
    def __init__(self):
        super().__init__('cylinder_position_controller')

        # シリンダロッドの寸法
        D_cyl = 0.020
        d_rod = 0.010
        self.AREA_HEAD = math.pi / 4.0 * D_cyl**2
        self.AREA_ROD = math.pi / 4.0 * (D_cyl**2 - d_rod**2)
        self.declare_parameter('ch_head', 0)
        self.declare_parameter('ch_rod', 1)
        self.CH_HEAD = self.get_parameter('ch_head').value
        self.CH_ROD = self.get_parameter('ch_rod').value

        self.VALVE_NEUTRAL = 5.0

        # 軌道
        self.declare_parameter('sine_amplitude_m', 0.020)
        self.declare_parameter('sine_freq_hz', 0.5)
        self.declare_parameter('center_position_m', 0.060)

        # 圧力
        self.declare_parameter('base_pressure_kpa', 250.0)
        self.declare_parameter('supply_pressure_kpa', 600.0)

        # 位置ループPID
        self.declare_parameter('pos_kp', 2000.0)
        self.declare_parameter('pos_ki', 500.0)
        self.declare_parameter('pos_kd', 100.0)
        self.declare_parameter('pos_td', 0.005)

        # 圧力ループPI
        self.declare_parameter('pres_kp', 0.02)
        self.declare_parameter('pres_ki', 0.005)
        self.declare_parameter('pres_kd', 0.0)
        self.declare_parameter('pres_td', 0.01)

        # ホーミング
        self.declare_parameter('homing_settle_threshold', 0.0002)
        self.declare_parameter('homing_settle_duration', 1.0)
        self.declare_parameter('homing_startup_wait', 0.5)

        # PRESSURIZE → RUNNING 遷移
        self.declare_parameter('pressurize_pos_threshold', 0.002)
        self.declare_parameter('pressurize_settle_duration', 0.5)

        # 状態変数
        self.state = ControllerState.WAITING_SENSOR
        self.x_0 = 0.0
        self.current_pos = None
        self.current_pH = 0.0
        self.current_pR = 0.0

        # ホーミング用
        self.homing_start_time = None
        self.homing_last_pos = None
        self.homing_settle_start = None

        # PRESSURIZE
        self.pressurize_settle_start = None

        self.run_start_time = None
        self.last_time = None

        # PID
        self.pid_pos = PIDController()
        self.pid_pH = PIDController()
        self.pid_pR = PIDController()

        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)
        self.pub_debug = self.create_publisher(
            Float32MultiArray, '/debug/cylinder_controller', 10)

        self.create_subscription(
            Float32, '/sensors/cylinder_position', self._cb_pos, 10)
        self.create_subscription(
            Float32, '/sensors/head_pressure', self._cb_ph, 10)
        self.create_subscription(
            Float32, '/sensors/rod_pressure', self._cb_pr, 10)

        self.timer = self.create_timer(0.01, self._control_loop)
        self.get_logger().info("Controller initialized. Waiting for sensors...")

    # センサコールバック
    def _cb_pos(self, msg):
        self.current_pos = msg.data

    def _cb_ph(self, msg):
        self.current_pH = msg.data

    def _cb_pr(self, msg):
        self.current_pR = msg.data

    # ユーティリティ
    def _send_valve(self, volt_h, volt_r):
        volt_h = max(0.0, min(10.0, volt_h))
        volt_r = max(0.0, min(10.0, volt_r))
        msg = Float32MultiArray()
        msg.data = [self.VALVE_NEUTRAL] * 8
        msg.data[self.CH_HEAD] = volt_h
        msg.data[self.CH_ROD] = volt_r
        self.pub_valve.publish(msg)

    def _send_all_neutral(self):
        self._send_valve(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)

    def _update_gains(self):
        self.pid_pos.kp = self.get_parameter('pos_kp').value
        self.pid_pos.ki = self.get_parameter('pos_ki').value
        self.pid_pos.kd = self.get_parameter('pos_kd').value
        self.pid_pos.td = self.get_parameter('pos_td').value

        for pid in [self.pid_pH, self.pid_pR]:
            pid.kp = self.get_parameter('pres_kp').value
            pid.ki = self.get_parameter('pres_ki').value
            pid.kd = self.get_parameter('pres_kd').value
            pid.td = self.get_parameter('pres_td').value

    def _get_relative_pos(self):
        return self.current_pos - self.x_0

    # 状態遷移メインループ
    def _control_loop(self):
        if self.state == ControllerState.WAITING_SENSOR:
            self._state_waiting_sensor()
        elif self.state == ControllerState.HOMING:
            self._state_homing()
        elif self.state == ControllerState.PRESSURIZE:
            self._state_pressurize()
        elif self.state == ControllerState.RUNNING:
            self._state_running()
        else:
            self._send_all_neutral()

    # 各状態の処理
    def _state_waiting_sensor(self):
        self._send_all_neutral()
        if self.current_pos is not None:
            self.get_logger().info("Sensors connected. Starting homing...")
            self.state = ControllerState.HOMING
            self.homing_start_time = time.time()
            self.homing_last_pos = self.current_pos
            self.homing_settle_start = None

    def _state_homing(self):
        settle_thresh = self.get_parameter('homing_settle_threshold').value
        settle_dur = self.get_parameter('homing_settle_duration').value
        startup_wait = self.get_parameter('homing_startup_wait').value

        self._send_valve(0.0, 10.0)

        now = time.time()

        if now - self.homing_start_time < startup_wait:
            self.homing_last_pos = self.current_pos
            return

        pos_change = abs(self.current_pos - self.homing_last_pos)

        if pos_change < settle_thresh:
            if self.homing_settle_start is None:
                self.homing_settle_start = now
            elif now - self.homing_settle_start > settle_dur:
                self.x_0 = self.current_pos
                self.get_logger().info(
                    f"Homing complete. Origin = {self.x_0:.4f} m")
                self.state = ControllerState.PRESSURIZE
                self.pressurize_settle_start = None
                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self.last_time = time.time()
                return
        else:
            self.homing_settle_start = None

        self.homing_last_pos = self.current_pos

    def _state_pressurize(self):
        """
        位置制御で中心位置まで移動。
        目標位置付近に安定したらRUNNINGに遷移。
        """
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now
        self._update_gains()

        center = self.get_parameter('center_position_m').value
        pos_thresh = self.get_parameter('pressurize_pos_threshold').value
        settle_dur = self.get_parameter('pressurize_settle_duration').value

        x_rel = self._get_relative_pos()
        target_force = self.pid_pos.update(center, x_rel, dt)
        self._apply_force_to_valves(target_force, dt)

        # 安定判定
        if abs(x_rel - center) < pos_thresh:
            if self.pressurize_settle_start is None:
                self.pressurize_settle_start = now
            elif now - self.pressurize_settle_start > settle_dur:
                self.get_logger().info(
                    f"Center reached ({x_rel * 1000:.1f} mm). Starting sine wave.")
                self.state = ControllerState.RUNNING
                self.run_start_time = time.time()
                self.pid_pos.reset()
                self.pid_pH.reset()
                self.pid_pR.reset()
                self.last_time = time.time()
                return
        else:
            self.pressurize_settle_start = None

    def _state_running(self):
        """
        正弦波追従制御。
        外側ループ: 位置PID → 目標推力
        内側ループ: 圧力PI → バルブ電圧
        PAMの反力は時変外乱としてPIDが吸収
        """
        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return
        self.last_time = now
        self._update_gains()

        amp = self.get_parameter('sine_amplitude_m').value
        freq = self.get_parameter('sine_freq_hz').value
        center = self.get_parameter('center_position_m').value

        elapsed = now - self.run_start_time
        omega = 2.0 * math.pi * freq

        # 目標軌道
        x_ref = center + amp * math.sin(omega * elapsed)

        # 位置PID → 目標推力
        x_rel = self._get_relative_pos()
        target_force = self.pid_pos.update(x_ref, x_rel, dt)

        # 推力 → 圧力 → バルブ
        self._apply_force_to_valves(target_force, dt)

        # デバッグ出力
        # msg = Float32MultiArray()
        # msg.data = [
        #     float(x_ref),                # [0] 目標位置 [m]
        #     float(x_rel),                # [1] 現在位置 [m]
        #     float(x_ref - x_rel),        # [2] 位置偏差 [m]
        #     float(target_force),         # [3] 目標推力 [N]
        #     float(self.current_pH),      # [4] ヘッド圧 [kPa]
        #     float(self.current_pR),      # [5] ロッド圧 [kPa]
        #     float(freq),                 # [6] 現在の周波数 [Hz]
        # ]
        # self.pub_debug.publish(msg)

    # 推力→圧力→バルブ変換
    def _apply_force_to_valves(self, target_force_N, dt):
        """
        目標推力を両室の圧力指令に変換し、
        圧力PIでバルブ電圧を算出して送信する。

        バイアス圧を両室にかけた状態を基準とし、
        目標力に応じて一方を加圧・他方を減圧する。
        """
        P_bias = self.get_parameter('base_pressure_kpa').value
        P_supply = self.get_parameter('supply_pressure_kpa').value

        # N → kPa: F[N] / A[m^2] = [Pa], /1000 → [kPa]
        P_bias_H = self.get_parameter('base_pressure_kpa').value
        # ロッド側は面積が小さい分、ベース圧力を高めにして釣り合わせる
        P_bias_R = P_bias_H * (self.AREA_HEAD / self.AREA_ROD)

        # N → kPa: F[N] / A[m^2] = [Pa], /1000 → [kPa]
        target_pH = P_bias_H + (target_force_N / self.AREA_HEAD / 1000.0 / 2.0)
        target_pR = P_bias_R - (target_force_N / self.AREA_ROD / 1000.0 / 2.0)

        # クランプ
        target_pH = max(0.0, min(P_supply, target_pH))
        target_pR = max(0.0, min(P_supply, target_pR))

        # 圧力PI → バルブ電圧
        uH = self.pid_pH.update(target_pH, self.current_pH, dt)
        uR = self.pid_pR.update(target_pR, self.current_pR, dt)

        self._send_valve(self.VALVE_NEUTRAL + uH, self.VALVE_NEUTRAL + uR)
        msg_debug = Float32MultiArray()
        msg_debug.data = [
            float(target_pH),       # [0] 目標ヘッド圧 (kPa)
            float(self.current_pH), # [1] 現在ヘッド圧 (kPa)
            float(target_pR),       # [2] 目標ロッド圧 (kPa)
            float(self.current_pR), # [3] 現在ロッド圧 (kPa)
            float(uH)               # [4] バルブ加算電圧 (V)
        ]
        self.pub_debug.publish(msg_debug)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node._send_all_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
