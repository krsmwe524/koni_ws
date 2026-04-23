#!/usr/bin/env python3
"""
MPYE サーボバルブ中立電圧スイープノード。

指定範囲の電圧を往復スイープし、各ステップでの流量計生電圧を記録する。
流量がゼロ（最小）になる電圧 = バルブの中立電圧。

出力:
  /actuators/valve_voltage : Float32MultiArray [8ch分の電圧]
入力:
  /ai1616llpe/voltage      : Float32MultiArray [AIボード生電圧]
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32


class SweepNode(Node):

    NEUTRAL_DEFAULT = 5.0  # スイープ外チャンネルの初期電圧

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

        self.valve_ch = self.get_parameter('valve_channel').value
        self.flow_ch = self.get_parameter('flowmeter_channel').value
        self.v_start = self.get_parameter('v_start').value
        self.v_end = self.get_parameter('v_end').value
        self.v_step = self.get_parameter('v_step').value
        self.hold_time = self.get_parameter('hold_time_s').value
        self.rate_hz = self.get_parameter('control_rate_hz').value

        # スイープ電圧列を生成（往復）
        self.voltage_list = self._build_sweep()
        self.step_index = 0
        self.hold_elapsed = 0.0
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

        self.create_subscription(
            Float32MultiArray, '/ai1616llpe/voltage', self._cb_ai, 10)

        dt = 1.0 / self.rate_hz
        self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f"Sweep: {self.v_start:.3f} -> {self.v_end:.3f} -> {self.v_start:.3f} V, "
            f"step={self.v_step:.3f} V, hold={self.hold_time:.1f} s, "
            f"valve_ch={self.valve_ch}, flow_ch={self.flow_ch}, "
            f"total {len(self.voltage_list)} steps"
        )

    # ------------------------------------------------------------------
    def _build_sweep(self):
        """往復の電圧リストを構築する。"""
        steps_forward = []
        v = self.v_start
        while v <= self.v_end + 1e-9:
            steps_forward.append(round(v, 4))
            v += self.v_step

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
        current_v = self.voltage_list[self.step_index]

        # バルブ電圧を出力 (8ch分、対象ch以外は中立)
        valve_msg = Float32MultiArray()
        voltages = [self.NEUTRAL_DEFAULT] * 8
        voltages[self.valve_ch] = current_v
        valve_msg.data = voltages
        self.pub_valve.publish(valve_msg)

        # デバッグ出力
        self.pub_debug_voltage.publish(Float32(data=current_v))
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
                valve_msg = Float32MultiArray()
                valve_msg.data = [self.NEUTRAL_DEFAULT] * 8
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
        msg = Float32MultiArray()
        msg.data = [SweepNode.NEUTRAL_DEFAULT] * 8
        node.pub_valve.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
