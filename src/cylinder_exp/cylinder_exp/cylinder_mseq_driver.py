import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Int32
from scipy.signal import max_len_seq
import numpy as np
import time


class CylinderMSeqDriver(Node):
    """
    シリンダ2弁を M系列で差動駆動する開ループドライバ。

        V_head(t) = 5.0 + A * m(t)
        V_rod(t)  = 5.0 - A * m(t)
        m(t) ∈ {-1, +1}  (M系列, クロック周期 T_c で更新)

    出力:
      /actuators/cylinder_valves : Float32MultiArray [ch_head, V_head, ch_rod, V_rod]
      /debug/mseq_value          : Float32 (現在の m(t))
      /debug/mseq_cycle_index    : Int32   (M系列の周回回数)
      /debug/mseq_amplitude_v    : Float32 (現在適用中の振幅 A)
      /debug/cylinder_v_head     : Float32
      /debug/cylinder_v_rod      : Float32
    """

    VALVE_NEUTRAL = 5.0

    def __init__(self):
        super().__init__('cylinder_mseq_driver')

        # ── パラメータ宣言 ────────────────────────────────────────
        self.declare_parameter('ch_head', 1)
        self.declare_parameter('ch_rod',  3)

        self.declare_parameter('amplitude_v',         1.0)
        self.declare_parameter('mseq_order',          12)
        self.declare_parameter('mseq_clock_period_s', 0.010)
        self.declare_parameter('mseq_seed',           1)

        self.declare_parameter('update_rate_hz',      1000.0)
        self.declare_parameter('startup_wait_s',      3.0)
        self.declare_parameter('startup_voltage_v',   self.VALVE_NEUTRAL)
        self.declare_parameter('amp_ramp_duration_s', 1.0)

        self.declare_parameter('loop_sequence', True)
        self.declare_parameter('max_cycles',    0)  # 0 = 無制限

        # ── パラメータ読み込み ────────────────────────────────────
        self.CH_HEAD = int(self.get_parameter('ch_head').value)
        self.CH_ROD  = int(self.get_parameter('ch_rod').value)

        self.A_target        = float(self.get_parameter('amplitude_v').value)
        self.n               = int(self.get_parameter('mseq_order').value)
        self.T_c             = float(self.get_parameter('mseq_clock_period_s').value)
        seed                 = int(self.get_parameter('mseq_seed').value)

        update_hz            = float(self.get_parameter('update_rate_hz').value)
        self.startup_wait_s  = float(self.get_parameter('startup_wait_s').value)
        self.startup_voltage = float(self.get_parameter('startup_voltage_v').value)
        self.ramp_duration_s = float(self.get_parameter('amp_ramp_duration_s').value)

        self.loop_seq        = bool(self.get_parameter('loop_sequence').value)
        self.max_cycles      = int(self.get_parameter('max_cycles').value)

        # ── M系列生成 ────────────────────────────────────────────
        state = self._make_state(seed, self.n)
        seq, _ = max_len_seq(self.n, state=state)
        # {0, 1} → {-1, +1}
        self.mseq = (2 * seq.astype(np.int8) - 1).astype(np.int8)
        self.seq_len = int(len(self.mseq))

        # ── 状態変数 ──────────────────────────────────────────────
        self.start_time = None
        self.stopped    = False

        # ── パブリッシャ ──────────────────────────────────────────
        self.pub_valve  = self.create_publisher(
            Float32MultiArray, '/actuators/cylinder_valves', 10)
        self.pub_mseq   = self.create_publisher(Float32, '/debug/mseq_value', 10)
        self.pub_cycle  = self.create_publisher(Int32,   '/debug/mseq_cycle_index', 10)
        self.pub_amp    = self.create_publisher(Float32, '/debug/mseq_amplitude_v', 10)
        self.pub_v_head = self.create_publisher(Float32, '/debug/cylinder_v_head', 10)
        self.pub_v_rod  = self.create_publisher(Float32, '/debug/cylinder_v_rod', 10)

        # ── タイマ ────────────────────────────────────────────────
        self.create_timer(1.0 / update_hz, self._update)

        self.get_logger().info(
            f"CylinderMSeqDriver started. "
            f"n={self.n}, len={self.seq_len}, T_c={self.T_c*1000:.1f}ms, "
            f"A={self.A_target:.3f}V, ch_head={self.CH_HEAD}, ch_rod={self.CH_ROD}, "
            f"seed={seed}, cycle_duration={self.seq_len * self.T_c:.2f}s, "
            f"loop={self.loop_seq}, max_cycles={self.max_cycles}, "
            f"startup_wait={self.startup_wait_s:.1f}s, "
            f"startup_voltage={self.startup_voltage:.3f}V"
        )

    # ── ユーティリティ ───────────────────────────────────────────
    @staticmethod
    def _make_state(seed, n):
        """seed (int) から n ビットの初期状態 (0/1 配列) を作成。全ゼロは禁止。"""
        rng = np.random.default_rng(seed)
        state = rng.integers(0, 2, size=n).astype(np.int8)
        if state.sum() == 0:
            state[0] = 1
        return state

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def _send_valve(self, v_head, v_rod):
        v_head = self._clamp(v_head, 0.0, 10.0)
        v_rod  = self._clamp(v_rod,  0.0, 10.0)

        msg = Float32MultiArray()
        msg.data = [
            float(self.CH_HEAD), float(v_head),
            float(self.CH_ROD),  float(v_rod),
        ]
        self.pub_valve.publish(msg)
        self.pub_v_head.publish(Float32(data=float(v_head)))
        self.pub_v_rod.publish(Float32(data=float(v_rod)))

    def _send_neutral(self):
        self._send_valve(self.VALVE_NEUTRAL, self.VALVE_NEUTRAL)

    def _send_startup_voltage(self):
        self._send_valve(self.startup_voltage, self.startup_voltage)

    # ── メインループ ─────────────────────────────────────────────
    def _update(self):
        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time

        # スタートアップ待機（PAM圧力安定化用）
        if elapsed < self.startup_wait_s:
            self._send_startup_voltage()
            self.pub_mseq.publish(Float32(data=0.0))
            self.pub_cycle.publish(Int32(data=0))
            self.pub_amp.publish(Float32(data=0.0))
            return

        if self.stopped:
            self._send_neutral()
            return

        t_seq     = elapsed - self.startup_wait_s
        idx_total = int(t_seq / self.T_c)
        cycle     = idx_total // self.seq_len

        # 終了判定
        if (self.max_cycles > 0 and cycle >= self.max_cycles) or \
           (not self.loop_seq and cycle >= 1):
            self.stopped = True
            self.get_logger().info(
                f"M-sequence finished after {cycle} cycle(s). "
                f"Holding neutral output."
            )
            self._send_neutral()
            return

        idx = idx_total % self.seq_len
        m   = float(self.mseq[idx])

        # 振幅ランプアップ（M系列開始から ramp_duration_s かけて 0 → A_target）
        if self.ramp_duration_s > 0.0 and t_seq < self.ramp_duration_s:
            A = self.A_target * (t_seq / self.ramp_duration_s)
        else:
            A = self.A_target

        v_head = self.VALVE_NEUTRAL + A * m
        v_rod  = self.VALVE_NEUTRAL - A * m

        self._send_valve(v_head, v_rod)
        self.pub_mseq.publish(Float32(data=m))
        self.pub_cycle.publish(Int32(data=int(cycle)))
        self.pub_amp.publish(Float32(data=float(A)))


def main(args=None):
    rclpy.init(args=args)
    node = CylinderMSeqDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        msg = Float32MultiArray()
        msg.data = [
            float(node.CH_HEAD), CylinderMSeqDriver.VALVE_NEUTRAL,
            float(node.CH_ROD),  CylinderMSeqDriver.VALVE_NEUTRAL,
        ]
        node.pub_valve.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
