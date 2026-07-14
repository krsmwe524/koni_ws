#!/usr/bin/env python3
"""
MPYE サーボバルブ電圧スイープノード。

指定範囲の電圧をスイープし、各ステップでの流量計生電圧を記録する。
必要に応じて、測定ステップ間に中立電圧でのタンク充填時間を設ける。

出力:
  /actuators/valve_voltage : Float32MultiArray [8ch分の電圧]
入力:
  /ai1616llpe/voltage      : Float32MultiArray [AIボード生電圧]
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32


class SweepNode(Node):
    def __init__(self):
        super().__init__('valve_sweep_node')

        # --- パラメータ ---
        self.declare_parameter('valve_channel', 1)
        self.declare_parameter('flowmeter_channel', 6)
        self.declare_parameter('v_start', 4.5)
        self.declare_parameter('v_end', 5.5)
        self.declare_parameter('v_step', 0.05)
        self.declare_parameter('hold_time_s', 2.0)
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('one_way', False)
        self.declare_parameter('neutral_voltage', 5.0)
        self.declare_parameter('recharge_between_steps', False)
        self.declare_parameter('recharge_time_s', 0.0)

        self.valve_ch = self.get_parameter('valve_channel').value
        self.flow_ch = self.get_parameter('flowmeter_channel').value
        self.v_start = self.get_parameter('v_start').value
        self.v_end = self.get_parameter('v_end').value
        self.v_step = self.get_parameter('v_step').value
        self.hold_time = self.get_parameter('hold_time_s').value
        self.rate_hz = self.get_parameter('control_rate_hz').value
        self.one_way = self.get_parameter('one_way').value
        self.neutral_voltage = self.get_parameter('neutral_voltage').value
        self.recharge_between_steps = self.get_parameter(
            'recharge_between_steps').value
        self.recharge_time = self.get_parameter('recharge_time_s').value

        self._validate_parameters()

        # スイープ電圧列を生成
        self.voltage_list = self._build_sweep()
        self.step_index = 0
        self.hold_elapsed = 0.0
        self.recharge_elapsed = 0.0
        self.recharging = False
        self.finished = False

        # 流量計の現在値
        self.flow_voltage = None

        # 各ステップでの流量サンプル蓄積（平均算出用）
        self.flow_samples = []

        # --- パブリッシャ / サブスクライバ ---
        self.pub_valve = self.create_publisher(
            Float32MultiArray, '/actuators/valve_voltage', 10)
        self.pub_debug_voltage = self.create_publisher(
            Float32, '/debug/sweep_voltage_V', 10)
        self.pub_debug_flow = self.create_publisher(
            Float32, '/debug/sweep_flow_raw_V', 10)
        self.pub_debug_measuring = self.create_publisher(
            Bool, '/debug/sweep_is_measuring', 10)
        self.pub_debug_step = self.create_publisher(
            Int32, '/debug/sweep_step_index', 10)

        self.create_subscription(
            Float32MultiArray, '/ai1616llpe/voltage', self._cb_ai, 10)

        dt = 1.0 / self.rate_hz
        self.create_timer(dt, self._control_loop)

        sweep_mode = 'one-way' if self.one_way else 'round-trip'
        sweep_path = (f"{self.v_start:.3f} -> {self.v_end:.3f} V"
                      if self.one_way
                      else f"{self.v_start:.3f} -> {self.v_end:.3f} -> {self.v_start:.3f} V")
        self.get_logger().info(
            f"Sweep ({sweep_mode}): {sweep_path}, "
            f"step={self.v_step:.3f} V, hold={self.hold_time:.1f} s, "
            f"recharge={self.recharge_time:.1f} s, "
            f"neutral={self.neutral_voltage:.3f} V, "
            f"valve_ch={self.valve_ch}, flow_ch={self.flow_ch}, "
            f"total {len(self.voltage_list)} steps"
        )

    # ------------------------------------------------------------------
    def _validate_parameters(self):
        """Fail early for parameters that could produce unsafe output."""
        if not 0 <= self.valve_ch < 8:
            raise ValueError('valve_channel must be in [0, 7]')
        if not 0 <= self.flow_ch < 16:
            raise ValueError('flowmeter_channel must be in [0, 15]')
        if self.rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        if self.v_step <= 0.0:
            raise ValueError('v_step must be positive')
        if self.hold_time <= 0.0:
            raise ValueError('hold_time_s must be positive')
        if self.recharge_time < 0.0:
            raise ValueError('recharge_time_s must be non-negative')

        for name, voltage in (
                ('v_start', self.v_start),
                ('v_end', self.v_end),
                ('neutral_voltage', self.neutral_voltage)):
            if not math.isfinite(voltage) or not 0.0 <= voltage <= 10.0:
                raise ValueError(f'{name} must be finite and in [0, 10] V')

    # ------------------------------------------------------------------
    def _build_sweep(self):
        """スイープ用の電圧リストを構築する。"""
        steps_forward = []
        v = self.v_start
        direction = 1.0 if self.v_end >= self.v_start else -1.0
        signed_step = direction * self.v_step

        def not_past_end(value):
            return direction * (self.v_end - value) >= -1e-9

        while not_past_end(v):
            steps_forward.append(round(v, 4))
            v += signed_step

        if self.one_way:
            return steps_forward

        # 往復: forward + reverse (両端の重複を除く)
        steps_reverse = list(reversed(steps_forward[:-1]))
        return steps_forward + steps_reverse

    # ------------------------------------------------------------------
    def _cb_ai(self, msg: Float32MultiArray):
        """AIボードの生電圧を受信。"""
        if self.flow_ch < len(msg.data):
            self.flow_voltage = float(msg.data[self.flow_ch])

    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.finished:
            return

        dt = 1.0 / self.rate_hz

        if self.recharging:
            self._publish_valve(self.neutral_voltage)
            self.pub_debug_voltage.publish(
                Float32(data=self.neutral_voltage))
            self.pub_debug_measuring.publish(Bool(data=False))
            self.pub_debug_step.publish(Int32(data=self.step_index))
            if self.flow_voltage is not None:
                self.pub_debug_flow.publish(Float32(data=self.flow_voltage))

            self.recharge_elapsed += dt
            if self.recharge_elapsed >= self.recharge_time:
                self.recharging = False
                self.recharge_elapsed = 0.0
                self.get_logger().info(
                    f"Recharge completed; starting step "
                    f"{self.step_index + 1}/{len(self.voltage_list)}."
                )
            return

        current_v = self.voltage_list[self.step_index]

        # バルブ電圧を出力 (8ch分、対象ch以外は中立)
        self._publish_valve(current_v)

        # デバッグ出力
        self.pub_debug_voltage.publish(Float32(data=current_v))
        self.pub_debug_measuring.publish(Bool(data=True))
        self.pub_debug_step.publish(Int32(data=self.step_index))
        if self.flow_voltage is not None:
            self.pub_debug_flow.publish(Float32(data=self.flow_voltage))
            self.flow_samples.append(self.flow_voltage)

        self.hold_elapsed += dt

        # ステップ完了判定
        if self.hold_elapsed >= self.hold_time:
            avg_flow = (sum(self.flow_samples) / len(self.flow_samples)
                        if self.flow_samples else float('nan'))
            self.get_logger().info(
                f"Step {self.step_index + 1}/{len(self.voltage_list)}: "
                f"V={current_v:.3f} V, flow_avg={avg_flow:.4f} V"
            )

            # 次のステップへ
            self.step_index += 1
            self.hold_elapsed = 0.0
            self.flow_samples = []

            if self.step_index >= len(self.voltage_list):
                self.get_logger().info("Sweep completed.")
                self.finished = True
                # 終了時は中立電圧に戻す
                self._publish_valve(self.neutral_voltage)
                self.pub_debug_measuring.publish(Bool(data=False))
            elif self.recharge_between_steps and self.recharge_time > 0.0:
                self.recharging = True
                self.get_logger().info(
                    f"Recharging at {self.neutral_voltage:.3f} V for "
                    f"{self.recharge_time:.1f} s."
                )

    def _publish_valve(self, selected_voltage):
        valve_msg = Float32MultiArray()
        voltages = [self.neutral_voltage] * 8
        voltages[self.valve_ch] = selected_voltage
        valve_msg.data = voltages
        self.pub_valve.publish(valve_msg)

    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = SweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        # 終了時にバルブを中立に戻す
        node._publish_valve(node.neutral_voltage)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
