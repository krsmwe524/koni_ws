#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from typing import List
import os
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SupplyValveFeatureNode(Node):
    """
    MPYE の給気側 (1→4) の流量特性を取得するための ROS2 ノード。

    - AOノード:
        /actuators/valve_voltage (Float32MultiArray)
        → 要素数1の [u] を publish すると、C++側で指定chに 0–10V が出力される想定。

    - AIノード:
        ai1616llpe/voltage (Float32MultiArray)
        → その data[ch] から流量・供給圧・下流圧を取得。

    流れ:
        u_list で与えた電圧を順に印加し、
        各 u について
            1) settling_duration [s] 待つ（遷移捨て）
            2) measure_duration [s] 計測
        する。

        計測中は「AIノードの update_rate」で呼ばれるコールバックごとに
            (time_from_start, step_index, u, psup_V, flow_V, pdown_V)
        を記録し、最後に CSV 出力する。
    """

    def __init__(self):
        super().__init__('supply_valve_feature_node')

        # ===== パラメータ定義 =====
        # 掃引するバルブ指令電圧 [V]
        self.declare_parameter('u_list', [5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0])
        # 中立電圧 [V]
        self.declare_parameter('u_neutral', 5.0)
        # 各ステップで遷移を捨てる時間 [s]
        self.declare_parameter('settling_duration', 1.0)
        # 各ステップで計測する時間 [s]
        self.declare_parameter('measure_duration', 1.0)

        # CSV 出力用のベース名（ディレクトリとタイムスタンプはコード側で付ける）
        self.declare_parameter('output_csv', 'supply_valve_feature')
        # ログを保存するかどうか
        self.declare_parameter('enable_logging', True)

        # トピック名
        self.declare_parameter('valve_cmd_topic', '/actuators/valve_voltage')
        # C++側が "ai1616llpe/voltage" を使っているのでそれに合わせる（先頭 / でもOK）
        self.declare_parameter('ai_topic', 'ai1616llpe/voltage')

        # AI チャンネル番号（0ベース）
        #  flow_ch  : 流量計
        #  psup_ch  : 供給圧 (上流圧)
        #  pdown_ch : 下流圧（今回は給気実験では使わなくてもよいが記録しておく）
        self.declare_parameter('flow_ch', 12)
        self.declare_parameter('psup_ch', 3)
        self.declare_parameter('pdown_ch', 2)

        # ===== パラメータ取得 =====
        self.u_list: List[float] = self.get_parameter('u_list').value
        self.u_neutral: float = float(self.get_parameter('u_neutral').value)
        self.settling_duration: float = float(self.get_parameter('settling_duration').value)
        self.measure_duration: float = float(self.get_parameter('measure_duration').value)

        base_name = self.get_parameter('output_csv').value
        self.enable_logging: bool = bool(self.get_parameter('enable_logging').value)

        valve_cmd_topic = self.get_parameter('valve_cmd_topic').value
        ai_topic = self.get_parameter('ai_topic').value

        self.flow_ch: int = int(self.get_parameter('flow_ch').value)
        self.psup_ch: int = int(self.get_parameter('psup_ch').value)
        self.pdown_ch: int = int(self.get_parameter('pdown_ch').value)

        # ==== 出力CSVパスを決定 (固定ディレクトリ + ベース名 + タイムスタンプ) ====
        log_dir = "/home/kklab/koni_ws/log"
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_csv: str = os.path.join(
            log_dir, f"{base_name}_{timestamp}.csv"
        )

        # ===== Publisher / Subscriber =====
        # AOボードへ：1要素 [u] を送る
        self.valve_pub = self.create_publisher(Float32MultiArray, valve_cmd_topic, 10)

        # AIボードから電圧配列を受け取る
        self.ai_sub = self.create_subscription(
            Float32MultiArray,
            ai_topic,
            self._ai_callback,
            10
        )

        # 最新センサ値バッファ
        self.latest_flow: float = 0.0
        self.latest_psup: float = 0.0
        self.latest_pdown: float = 0.0

        # ===== 実験状態管理 =====
        self.start_time = None           # 実験開始時刻（最初のAIメッセージ到着でセット）
        self.current_step = -1           # どの u_list のインデックスか
        self.phase = 'init'              # 'init' | 'settling' | 'measuring' | 'done'
        self.phase_start_time = None
        self.current_u = self.u_neutral

        # 記録バッファ:
        # (time_from_start_s, step_index, u_V, psup_V, flow_V, pdown_V)
        self.records: List[tuple] = []

        # 中立電圧で開始（安全のため）
        self._publish_valve_voltage(self.u_neutral)

        self.get_logger().info(
            "SupplyValveFeatureNode started.\n"
            f"  u_list={self.u_list}\n"
            f"  settling={self.settling_duration}s, measure={self.measure_duration}s\n"
            f"  flow_ch={self.flow_ch}, psup_ch={self.psup_ch}, pdown_ch={self.pdown_ch}\n"
            f"  enable_logging={self.enable_logging}\n"
            f"  output_csv='{self.output_csv}'\n"
            f"  ai_topic='{ai_topic}', valve_cmd_topic='{valve_cmd_topic}'"
        )

    # センサコールバック
    def _ai_callback(self, msg: Float32MultiArray):
        """AIボードからの電圧を受信するたびに呼ばれる。ここでフェーズ管理+ログも行う"""
        data = msg.data
        n = len(data)

        # センサ値更新
        if self.flow_ch < n:
            self.latest_flow = float(data[self.flow_ch])
        if self.psup_ch < n:
            self.latest_psup = float(data[self.psup_ch])
        if self.pdown_ch < n:
            self.latest_pdown = float(data[self.pdown_ch])

        now = self.get_clock().now().nanoseconds / 1e9

        # 最初のメッセージ時に start_time / phase_start_time をセット
        if self.start_time is None:
            self.start_time = now
            self.phase_start_time = now

        t_from_start = now - self.start_time
        elapsed_in_phase = now - self.phase_start_time

        # === フェーズ管理 ===

        # 初回：最初のステップに入る
        if self.phase == 'init':
            self.current_step = 0
            if self.current_step >= len(self.u_list):
                self._finish_experiment()
                return

            self.current_u = float(self.u_list[self.current_step])
            self._publish_valve_voltage(self.current_u)
            self.phase = 'settling'
            self.phase_start_time = now
            self.get_logger().info(
                f"[step {self.current_step}] u={self.current_u:.3f} V : settling start"
            )
            return

        if self.phase == 'done':
            return

        if self.phase == 'settling':
            # 遷移中: 何もしないで待つ
            if elapsed_in_phase >= self.settling_duration:
                # 測定フェーズへ移行
                self.phase = 'measuring'
                self.phase_start_time = now
                self.get_logger().info(
                    f"[step {self.current_step}] u={self.current_u:.3f} V : measuring start"
                )
            return

        if self.phase == 'measuring':
            # 計測フェーズ: AIの update_rate で呼ばれる＝そのまま記録周波数になる
            if self.enable_logging:
                self.records.append(
                    (
                        t_from_start,
                        int(self.current_step),
                        float(self.current_u),
                        float(self.latest_psup),
                        float(self.latest_flow),
                        float(self.latest_pdown),
                    )
                )

            if elapsed_in_phase >= self.measure_duration:
                self.get_logger().info(
                    f"[step {self.current_step}] measuring done "
                    f"(duration={self.measure_duration}s)"
                )

                # 次のステップへ
                self.current_step += 1
                if self.current_step >= len(self.u_list):
                    # 全ステップ完了
                    self._finish_experiment()
                    return

                # 次の電圧に切り替え
                self.current_u = float(self.u_list[self.current_step])
                self._publish_valve_voltage(self.current_u)
                self.phase = 'settling'
                self.phase_start_time = now
                self.get_logger().info(
                    f"[step {self.current_step}] u={self.current_u:.3f} V : settling start"
                )
            return

        # それ以外は想定外
        self.get_logger().warn(f"Unknown phase: {self.phase}")

    # ===== AO 出力 =====

    def _publish_valve_voltage(self, u: float):
        """
        AOノードは:
          - 要素数1 → 'channel' パラメータで指定されたchのみ更新
        という仕様なので、ここでは [u] だけ送る。
        """
        msg = Float32MultiArray()
        msg.data = [float(u)]
        self.valve_pub.publish(msg)

    # ===== 終了処理 =====

    def _finish_experiment(self):
        """中立電圧に戻し、必要なら CSV を書き出して終了ステートに入る。"""
        if self.phase == 'done':
            return

        self.get_logger().info(
            "All steps finished. Returning valve to neutral and finalizing."
        )
        self.phase = 'done'

        # 中立電圧に戻す
        self._publish_valve_voltage(self.u_neutral)

        # ログ保存が有効なら CSV 書き出し
        if self.enable_logging:
            try:
                with open(self.output_csv, mode='w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(
                        ['time_from_start_s', 'step_index', 'u_V',
                         'psup_V', 'flow_V', 'pdown_V']
                    )
                    for row in self.records:
                        writer.writerow(row)
                self.get_logger().info(
                    f"Saved {len(self.records)} samples to: {self.output_csv}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to write CSV: {e}")
        else:
            self.get_logger().info("enable_logging=false: CSV 出力はスキップしました。")

        self.get_logger().info("Experiment finished. You can Ctrl+C to exit this node.")


def main(args=None):
    rclpy.init(args=args)
    node = SupplyValveFeatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "KeyboardInterrupt: returning valve to neutral and shutting down."
        )
        node._publish_valve_voltage(node.u_neutral)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()