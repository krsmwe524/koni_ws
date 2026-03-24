# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import random
# import time


# class PRBSGeneratorNode(Node):
#     def __init__(self):
#         super().__init__("prbs_generator")

#         # ---------------- Parameters ----------------
#         # Hold interval
#         self.declare_parameter("hold_dt_mode", "fixed")  # fixed / discrete / uniform
#         self.declare_parameter("hold_dt_fixed", 0.3)

#         self.declare_parameter("hold_dt_min", 0.1)
#         self.declare_parameter("hold_dt_max", 1.0)
#         self.declare_parameter("hold_dt_levels", [0.3])
#         self.declare_parameter("hold_dt_weights", [])

#         # Voltage levels (10 values)
#         # NOTE: 5.0(中立)は levels に入れない（排気/給気のside判定を明確にするため）
#         self.declare_parameter(
#             "levels",
#             [3.0, 3.4, 3.8, 4.2, 4.6, 5.4, 5.8, 6.2, 6.6, 7.0],
#         )

#         # Timeline
#         self.declare_parameter("start_neutral_sec", 3.0)
#         self.declare_parameter("washout_sec", 3.0)
#         self.declare_parameter("duration_sec", 180.0)
#         self.declare_parameter("end_neutral_sec", 2.0)
#         self.declare_parameter("neutral_v", 5.0)

#         # RNG
#         self.declare_parameter("seed", 42)

#         # Constraint: prevent >=4 consecutive steps on the same side
#         # max_same_side_run=3 => 4回目は必ず反対側にする
#         self.declare_parameter("max_same_side_run", 3)

#         # L/R mode
#         self.declare_parameter("same_lr", True)

#         # ---------------- Read parameters ----------------
#         self.hold_dt_mode = self.get_parameter("hold_dt_mode").value
#         self.hold_dt_fixed = float(self.get_parameter("hold_dt_fixed").value)
#         self.hold_dt_min = float(self.get_parameter("hold_dt_min").value)
#         self.hold_dt_max = float(self.get_parameter("hold_dt_max").value)
#         self.hold_dt_levels = [float(x) for x in self.get_parameter("hold_dt_levels").value]
#         self.hold_dt_weights = list(self.get_parameter("hold_dt_weights").value)

#         self.levels = [float(x) for x in self.get_parameter("levels").value]
#         self.neutral_v = float(self.get_parameter("neutral_v").value)

#         self.start_neutral_sec = float(self.get_parameter("start_neutral_sec").value)
#         self.washout_sec = float(self.get_parameter("washout_sec").value)
#         self.duration_sec = float(self.get_parameter("duration_sec").value)
#         self.end_neutral_sec = float(self.get_parameter("end_neutral_sec").value)

#         self.seed = int(self.get_parameter("seed").value)
#         self.max_same_side_run = int(self.get_parameter("max_same_side_run").value)
#         self.same_lr = bool(self.get_parameter("same_lr").value)

#         # ---------------- RNG ----------------
#         self.rng_timing = random.Random(self.seed)
#         self.rng_level = random.Random(self.seed + 1)

#         # ---------------- State ----------------
#         self.start_time = self.get_clock().now()

#         self.current_u_left = self.neutral_v
#         self.current_u_right = self.neutral_v

#         # Hold dt
#         if self.hold_dt_mode == "fixed":
#             self.current_hold_dt = self.hold_dt_fixed
#         elif self.hold_dt_mode == "uniform":
#             self.current_hold_dt = self.hold_dt_min
#         else:
#             self.current_hold_dt = self.hold_dt_levels[0] if self.hold_dt_levels else self.hold_dt_min

#         # run tracking (global when same_lr=True)
#         self._last_side = None     # 'exhaust' or 'intake' or None
#         self._run_len = 0

#         # Timeline (t_exp in seconds)
#         self.t_washout = self.start_neutral_sec
#         self.t_main = self.t_washout + self.washout_sec
#         self.t_end_neutral = self.t_main + self.duration_sec
#         self.t_final = self.t_end_neutral + self.end_neutral_sec

#         self.next_switch_time = self.t_washout

#         # Publisher
#         self.pub_valve = self.create_publisher(Float32MultiArray, "/actuators/valve_voltage", 10)

#         # 1000 Hz loop (hold logic is separate)
#         self.timer = self.create_timer(0.001, self.timer_callback)

#         self.get_logger().info(
#             "PRBS Generator Started. "
#             f"hold_dt_mode={self.hold_dt_mode}, hold_dt_fixed={self.hold_dt_fixed}, "
#             f"levels={self.levels}, seed={self.seed}, same_lr={self.same_lr}, "
#             f"max_same_side_run={self.max_same_side_run}"
#         )

#     # ---------------- Utility ----------------
#     def _side_of_u(self, u: float) -> str:
#         if u < self.neutral_v:
#             return "exhaust"
#         if u > self.neutral_v:
#             return "intake"
#         return "neutral"

#     def _sample_hold_dt(self) -> float:
#         if self.hold_dt_mode == "fixed":
#             return float(self.hold_dt_fixed)
#         if self.hold_dt_mode == "uniform":
#             return float(self.rng_timing.uniform(self.hold_dt_min, self.hold_dt_max))
#         # discrete
#         levels = self.hold_dt_levels
#         if not levels:
#             return max(1e-3, self.hold_dt_min)
#         w = self.hold_dt_weights
#         if w and len(w) == len(levels):
#             return float(self.rng_timing.choices(levels, weights=w, k=1)[0])
#         return float(self.rng_timing.choice(levels))

#     def _sample_u_with_run_limit(self) -> float:
#         # 連続回数が max_same_side_run に達したら反対側からしか選ばない
#         candidates = self.levels

#         if self._last_side in ("exhaust", "intake") and self._run_len >= self.max_same_side_run:
#             if self._last_side == "exhaust":
#                 candidates = [v for v in self.levels if v > self.neutral_v]
#             else:
#                 candidates = [v for v in self.levels if v < self.neutral_v]
#             if not candidates:
#                 candidates = self.levels

#         return float(self.rng_level.choice(candidates))

#     def _update_run(self, u: float):
#         side = self._side_of_u(u)
#         if side == "neutral":
#             self._last_side = "neutral"
#             self._run_len = 1
#             return
#         if side == self._last_side:
#             self._run_len += 1
#         else:
#             self._last_side = side
#             self._run_len = 1

#     def _publish_current_u(self):
#         msg = Float32MultiArray()
#         msg.data = [float(self.current_u_left), float(self.current_u_right)]
#         self.pub_valve.publish(msg)

#     # ---------------- Main loop ----------------
#     def timer_callback(self):
#         now = self.get_clock().now()
#         t_exp = (now - self.start_time).nanoseconds / 1e9

#         if t_exp < self.t_washout:
#             self.current_u_left = self.neutral_v
#             self.current_u_right = self.neutral_v

#         elif t_exp < self.t_main:
#             self._update_prbs(t_exp)

#         elif t_exp < self.t_end_neutral:
#             self._update_prbs(t_exp)

#         elif t_exp < self.t_final:
#             self.current_u_left = self.neutral_v
#             self.current_u_right = self.neutral_v

#         else:
#             self.current_u_left = self.neutral_v
#             self.current_u_right = self.neutral_v
#             self._publish_current_u()
#             self.get_logger().info("Experiment Finished.")
#             self.timer.cancel()
#             self.destroy_node()
#             raise SystemExit

#         self._publish_current_u()

#     def _update_prbs(self, t_exp: float):
#         if t_exp < self.next_switch_time:
#             return

#         # sample next voltage
#         u = self._sample_u_with_run_limit()
#         self._update_run(u)

#         if self.same_lr:
#             self.current_u_left = u
#             self.current_u_right = u
#         else:
#             # もし将来左右独立にしたくなったらここを拡張
#             self.current_u_left = u
#             self.current_u_right = u

#         # sample next hold duration
#         self.current_hold_dt = self._sample_hold_dt()
#         self.next_switch_time = t_exp + self.current_hold_dt


# def main():
#     rclpy.init()
#     node = PRBSGeneratorNode()
#     try:
#         rclpy.spin(node)
#     except (KeyboardInterrupt, SystemExit):
#         final_node = rclpy.create_node("final_safety_pub")
#         pub = final_node.create_publisher(Float32MultiArray, "/actuators/valve_voltage", 10)
#         msg = Float32MultiArray()
#         msg.data = [5.0, 5.0]
#         pub.publish(msg)
#         final_node.get_logger().info("Safety voltage 5.0V published.")
#         time.sleep(0.1)
#         final_node.destroy_node()
#     finally:
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import random
import time


class PRBSGeneratorNode(Node):
    def __init__(self):
        super().__init__("prbs_generator")

        # ---------------- Parameters ----------------
        # Output topic (keep default)
        self.declare_parameter("output_topic", "/actuators/valve_voltage")

        # Hold interval
        self.declare_parameter("hold_dt_mode", "fixed")  # fixed / discrete / uniform
        self.declare_parameter("hold_dt_fixed", 0.3)

        self.declare_parameter("hold_dt_min", 0.1)
        self.declare_parameter("hold_dt_max", 1.0)
        self.declare_parameter("hold_dt_levels", [0.3])
        self.declare_parameter("hold_dt_weights", [])

        # Voltage levels (discrete values)
        # NOTE: 5.0(中立)は levels に入れない（side判定を明確にするため）
        self.declare_parameter(
            "levels",
            [3.0, 3.4, 3.8, 4.2, 4.6, 5.4, 5.8, 6.2, 6.6, 7.0],
        )

        # Timeline
        self.declare_parameter("start_neutral_sec", 3.0)
        self.declare_parameter("washout_sec", 3.0)
        self.declare_parameter("duration_sec", 180.0)
        self.declare_parameter("end_neutral_sec", 2.0)
        self.declare_parameter("neutral_v", 5.0)

        # RNG
        self.declare_parameter("seed", 42)

        # Constraint (optional): prevent >=(max_same_side_run+1) consecutive steps on the same side
        # max_same_side_run<=0 で無効化
        self.declare_parameter("max_same_side_run", 3)

        # L/R mode
        self.declare_parameter("same_lr", True)

        # ---- Pressure guard (NEW) ----
        # PAM2の圧（kPa）を監視して、範囲外なら neutral に戻す
        self.declare_parameter("enable_p2_guard", True)
        self.declare_parameter("p2_topic", "/sensors/R_pres_kpa")  # PAM2圧に合わせて変更
        self.declare_parameter("p2_min_kpa", 80.0)
        self.declare_parameter("p2_max_kpa", 240.0)

        # ---------------- Read parameters ----------------
        self.output_topic = self.get_parameter("output_topic").value

        self.hold_dt_mode = self.get_parameter("hold_dt_mode").value
        self.hold_dt_fixed = float(self.get_parameter("hold_dt_fixed").value)
        self.hold_dt_min = float(self.get_parameter("hold_dt_min").value)
        self.hold_dt_max = float(self.get_parameter("hold_dt_max").value)
        self.hold_dt_levels = [float(x) for x in self.get_parameter("hold_dt_levels").value]
        self.hold_dt_weights = list(self.get_parameter("hold_dt_weights").value)

        self.levels = [float(x) for x in self.get_parameter("levels").value]
        self.neutral_v = float(self.get_parameter("neutral_v").value)

        self.start_neutral_sec = float(self.get_parameter("start_neutral_sec").value)
        self.washout_sec = float(self.get_parameter("washout_sec").value)
        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.end_neutral_sec = float(self.get_parameter("end_neutral_sec").value)

        self.seed = int(self.get_parameter("seed").value)
        self.max_same_side_run = int(self.get_parameter("max_same_side_run").value)
        self.same_lr = bool(self.get_parameter("same_lr").value)

        self.enable_p2_guard = bool(self.get_parameter("enable_p2_guard").value)
        self.p2_topic = self.get_parameter("p2_topic").value
        self.p2_min_kpa = float(self.get_parameter("p2_min_kpa").value)
        self.p2_max_kpa = float(self.get_parameter("p2_max_kpa").value)

        # ---------------- I/O ----------------
        self.pub_valve = self.create_publisher(Float32MultiArray, self.output_topic, 10)

        # PAM2圧（kPa）購読（guard用）
        self.p2_kpa = None
        if self.enable_p2_guard:
            self.sub_p2 = self.create_subscription(Float32, self.p2_topic, self._cb_p2, 10)

        # ---------------- RNG ----------------
        self.rng_timing = random.Random(self.seed)
        self.rng_level = random.Random(self.seed + 1)

        # ---------------- State ----------------
        self.start_time = self.get_clock().now()

        self.current_u_left = self.neutral_v
        self.current_u_right = self.neutral_v

        # Hold dt
        if self.hold_dt_mode == "fixed":
            self.current_hold_dt = self.hold_dt_fixed
        elif self.hold_dt_mode == "uniform":
            self.current_hold_dt = self.hold_dt_min
        else:
            self.current_hold_dt = self.hold_dt_levels[0] if self.hold_dt_levels else self.hold_dt_min

        # run tracking (global when same_lr=True)
        self._last_side = None     # 'exhaust' or 'intake' or 'neutral' or None
        self._run_len = 0

        # Timeline (t_exp in seconds)
        self.t_washout = self.start_neutral_sec
        self.t_main = self.t_washout + self.washout_sec
        self.t_end_neutral = self.t_main + self.duration_sec
        self.t_final = self.t_end_neutral + self.end_neutral_sec

        self.next_switch_time = self.t_washout

        # 1000 Hz loop (hold logic is separate)
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.get_logger().info(
            "PRBS Generator Started. "
            f"output_topic={self.output_topic}, "
            f"hold_dt_mode={self.hold_dt_mode}, hold_dt_fixed={self.hold_dt_fixed}, "
            f"levels={self.levels}, seed={self.seed}, same_lr={self.same_lr}, "
            f"max_same_side_run={self.max_same_side_run}, "
            f"p2_guard={self.enable_p2_guard} [{self.p2_min_kpa}, {self.p2_max_kpa}] kPa on {self.p2_topic}"
        )

    # ---------------- Callbacks ----------------
    def _cb_p2(self, msg: Float32):
        try:
            self.p2_kpa = float(msg.data)
        except Exception:
            pass

    # ---------------- Utility ----------------
    def _side_of_u(self, u: float) -> str:
        if u < self.neutral_v:
            return "exhaust"
        if u > self.neutral_v:
            return "intake"
        return "neutral"

    def _sample_hold_dt(self) -> float:
        if self.hold_dt_mode == "fixed":
            return float(self.hold_dt_fixed)
        if self.hold_dt_mode == "uniform":
            return float(self.rng_timing.uniform(self.hold_dt_min, self.hold_dt_max))
        # discrete
        levels = self.hold_dt_levels
        if not levels:
            return max(1e-3, self.hold_dt_min)
        w = self.hold_dt_weights
        if w and len(w) == len(levels):
            return float(self.rng_timing.choices(levels, weights=w, k=1)[0])
        return float(self.rng_timing.choice(levels))

    def _sample_u_with_run_limit(self) -> float:
        # max_same_side_run<=0 なら連続禁止ルールを無効化
        if self.max_same_side_run <= 0:
            return float(self.rng_level.choice(self.levels))

        candidates = self.levels
        if self._last_side in ("exhaust", "intake") and self._run_len >= self.max_same_side_run:
            if self._last_side == "exhaust":
                candidates = [v for v in self.levels if v > self.neutral_v]
            else:
                candidates = [v for v in self.levels if v < self.neutral_v]
            if not candidates:
                candidates = self.levels

        return float(self.rng_level.choice(candidates))

    def _update_run(self, u: float):
        side = self._side_of_u(u)
        if side == "neutral":
            self._last_side = "neutral"
            self._run_len = 1
            return
        if side == self._last_side:
            self._run_len += 1
        else:
            self._last_side = side
            self._run_len = 1

    def _publish_current_u(self):
        msg = Float32MultiArray()
        msg.data = [float(self.current_u_left), float(self.current_u_right)]
        self.pub_valve.publish(msg)

    def _p2_out_of_range(self) -> bool:
        if not self.enable_p2_guard:
            return False
        if self.p2_kpa is None:
            return False  # 未受信の間は介入しない
        return (self.p2_kpa < self.p2_min_kpa) or (self.p2_kpa > self.p2_max_kpa)

    # ---------------- Main loop ----------------
    def timer_callback(self):
        now = self.get_clock().now()
        t_exp = (now - self.start_time).nanoseconds / 1e9

        if t_exp < self.t_washout:
            self.current_u_left = self.neutral_v
            self.current_u_right = self.neutral_v

        elif t_exp < self.t_main:
            self._update_prbs(t_exp)

        elif t_exp < self.t_end_neutral:
            self._update_prbs(t_exp)

        elif t_exp < self.t_final:
            self.current_u_left = self.neutral_v
            self.current_u_right = self.neutral_v

        else:
            self.current_u_left = self.neutral_v
            self.current_u_right = self.neutral_v
            self._publish_current_u()
            self.get_logger().info("Experiment Finished.")
            self.timer.cancel()
            self.destroy_node()
            raise SystemExit

        # ---- Pressure guard: immediate neutral override (NEW) ----
        if (t_exp >= self.t_washout) and (t_exp < self.t_end_neutral) and self._p2_out_of_range():
            self.current_u_left = self.neutral_v
            self.current_u_right = self.neutral_v
            # 連続禁止の状態もneutral扱いに戻しておく（副作用を減らす）
            self._last_side = "neutral"
            self._run_len = 1

        self._publish_current_u()

    def _update_prbs(self, t_exp: float):
        if t_exp < self.next_switch_time:
            return

        # sample next voltage
        u = self._sample_u_with_run_limit()
        self._update_run(u)

        if self.same_lr:
            self.current_u_left = u
            self.current_u_right = u
        else:
            self.current_u_left = u
            self.current_u_right = u

        # sample next hold duration
        self.current_hold_dt = self._sample_hold_dt()
        self.next_switch_time = t_exp + self.current_hold_dt


def main():
    rclpy.init()
    node = PRBSGeneratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        final_node = rclpy.create_node("final_safety_pub")
        pub = final_node.create_publisher(Float32MultiArray, "/actuators/valve_voltage", 10)
        msg = Float32MultiArray()
        msg.data = [5.0, 5.0]
        pub.publish(msg)
        final_node.get_logger().info("Safety voltage 5.0V published.")
        time.sleep(0.1)
        final_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()