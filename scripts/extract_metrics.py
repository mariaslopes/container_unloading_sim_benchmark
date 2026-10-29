#!/usr/bin/env python3
"""Unloading metrics for a mobile manipulator.

The run starts when the base first moves and ends when the base is back at its
start pose and has stopped. If that never happens, the run ends after an idle
timeout (nothing moved and the suction did not change) or on Ctrl+C.

Boxes are counted from the suction command: each ON -> OFF transition is one
box. Base and arm activity come from /emrox/odom and /joint_states, and
perception triggers from /d435/trigger (any message type).

At the end the node prints a report and writes report.txt, summary.json,
boxes.csv and timeline.png (if matplotlib is available) to a new folder in
output_dir.

Usage:
    python3 unloading_metrics.py --ros-args -p use_sim_time:=true
"""

import csv
import json
import math
import os
import statistics
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from rosidl_runtime_py.utilities import get_message
except ImportError:  # pragma: no cover
    get_message = None


UR_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def merge_intervals(intervals):
    merged = []
    for a, b in sorted(intervals):
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    return [tuple(m) for m in merged]


def total_duration(intervals):
    return sum(b - a for a, b in intervals)


def describe(values):
    if not values:
        return None
    return {
        "mean": statistics.mean(values),
        "median": statistics.median(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
    }


def mmss(seconds):
    m, s = divmod(seconds, 60.0)
    return f"{int(m):d} min {s:04.1f} s"


class ActivityTracker:
    """Turns a noisy moving/not-moving signal into activity intervals."""

    def __init__(self, hold_time):
        self.hold_time = hold_time
        self.active = False
        self.seg_start = None
        self.last_active = None
        self.segments = []

    def update(self, t, moving):
        if moving:
            if not self.active:
                self.active = True
                self.seg_start = t
            self.last_active = t
        elif self.active and t - self.last_active > self.hold_time:
            self.active = False
            self.segments.append((self.seg_start, self.last_active))

    def intervals(self, t0, t1):
        segments = list(self.segments)
        if self.active:
            segments.append((self.seg_start, self.last_active))
        out = []
        for a, b in segments:
            a, b = max(a, t0), min(b, t1)
            if b > a:
                out.append((a, b))
        return out


# --------------------------------------------------------------------------- #
# Node
# --------------------------------------------------------------------------- #
class UnloadingMetrics(Node):
    WAITING, RUNNING, FINISHED = range(3)

    def __init__(self):
        super().__init__("unloading_metrics")

        self.declare_parameter("odom_topic", "/emrox/odom")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("suction_topic", "/epick/suction")
        self.declare_parameter("trigger_topic", "/d435/trigger")
        self.declare_parameter("arm_joints", UR_JOINTS)
        self.declare_parameter("base_linear_threshold", 0.01)  # m/s
        self.declare_parameter("base_angular_threshold", 0.02)  # rad/s
        self.declare_parameter("arm_velocity_threshold", 0.01)  # rad/s
        self.declare_parameter("hold_time", 0.5)  # s
        self.declare_parameter("min_departure", 0.30)  # m
        self.declare_parameter("return_tolerance", 0.10)  # m
        self.declare_parameter("idle_timeout", 120.0)  # s
        self.declare_parameter("output_dir", "~/unloading_reports")

        p = lambda name: self.get_parameter(name).value
        self.arm_joints = list(p("arm_joints"))
        self.base_lin_thr = p("base_linear_threshold")
        self.base_ang_thr = p("base_angular_threshold")
        self.arm_thr = p("arm_velocity_threshold")
        self.min_departure = p("min_departure")
        self.return_tol = p("return_tolerance")
        self.idle_timeout = p("idle_timeout")
        self.output_dir = os.path.expanduser(p("output_dir"))
        self.trigger_topic = p("trigger_topic")

        hold = p("hold_time")
        self.base = ActivityTracker(hold)
        self.arm = ActivityTracker(hold)

        self.state = self.WAITING
        self.finished = False
        self.last_t = 0.0
        self.start_time = None
        self.end_time = None
        self.end_reason = ""

        self.start_pose = None
        self.base_xy = (0.0, 0.0)
        self.prev_odom = None
        self.base_distance = 0.0
        self.max_departure = 0.0

        self.prev_joints = None

        self.suction_on = False
        self.grasp_t = None
        self.grasp_offset = None
        self.suction_segments = []
        self.last_suction_t = None
        self.boxes = []

        self.trigger_times = []
        self.trigger_sub = None

        self.create_subscription(Odometry, p("odom_topic"), self.odom_cb, 50)
        self.create_subscription(
            JointState, p("joint_states_topic"), self.joints_cb, 50
        )
        self.create_subscription(Bool, p("suction_topic"), self.suction_cb, 10)
        self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("Waiting for the base to start moving")

    # ----------------------------------------------------------------------- #
    def now(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        if t > 0.0:
            self.last_t = t
        return t

    def start(self, t):
        self.state = self.RUNNING
        self.start_time = t
        self.get_logger().info("Base started moving, run started")

    def finish(self, t, reason):
        if self.state != self.RUNNING:
            return
        self.state = self.FINISHED
        self.end_time = t
        self.end_reason = reason
        self.get_logger().info(f"Run finished ({reason})")
        self.report()
        self.finished = True

    # ----------------------------------------------------------------------- #
    def odom_cb(self, msg):
        t = self.now()
        if t <= 0.0:
            return  # no /clock yet

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.start_pose is None:
            self.start_pose = (x, y, yaw)

        if self.prev_odom is not None:
            pt, px, py, pyaw = self.prev_odom
            dt = t - pt
            if dt <= 0.0:
                return
            step = math.hypot(x - px, y - py)
            moving = (
                step / dt > self.base_lin_thr
                or abs(angle_diff(yaw, pyaw)) / dt > self.base_ang_thr
            )
            self.base.update(t, moving)
            if self.state == self.RUNNING and moving:
                self.base_distance += step

        self.prev_odom = (t, x, y, yaw)
        self.base_xy = (x, y)

        if self.state == self.WAITING and self.base.active:
            self.start(self.base.seg_start)
            return

        if self.state == self.RUNNING:
            x0, y0, _ = self.start_pose
            dist = math.hypot(x - x0, y - y0)
            self.max_departure = max(self.max_departure, dist)
            back = (
                self.max_departure > self.min_departure
                and dist < self.return_tol
                and not self.base.active
                and self.base.segments
                and self.base.segments[-1][1] > self.start_time
            )
            if back:
                self.finish(
                    self.base.segments[-1][1], "base returned to the start pose"
                )

    def joints_cb(self, msg):
        t = self.now()
        if t <= 0.0:
            return
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            if any(name.endswith(j) for j in self.arm_joints):
                positions[name] = pos
        if not positions:
            return

        if self.prev_joints is not None:
            pt, prev = self.prev_joints
            dt = t - pt
            if dt <= 0.0:
                return
            speed = max(
                (abs(q - prev[n]) / dt for n, q in positions.items() if n in prev),
                default=0.0,
            )
            self.arm.update(t, speed > self.arm_thr)
        self.prev_joints = (t, positions)

    def suction_cb(self, msg):
        t = self.now()
        on = bool(msg.data)
        if on == self.suction_on:
            return
        self.suction_on = on
        self.last_suction_t = t

        if on:
            self.grasp_t = t
            if self.start_pose is not None:
                self.grasp_offset = math.hypot(
                    self.base_xy[0] - self.start_pose[0],
                    self.base_xy[1] - self.start_pose[1],
                )
            return

        if self.grasp_t is None:
            return
        self.suction_segments.append((self.grasp_t, t))
        if self.state == self.RUNNING and self.grasp_t >= self.start_time:
            ref = self.boxes[-1]["release_t"] if self.boxes else self.start_time
            self.boxes.append(
                {
                    "grasp_t": self.grasp_t,
                    "release_t": t,
                    "ref_t": ref,
                    "base_offset": self.grasp_offset,
                }
            )
            self.get_logger().info(
                f"Box {len(self.boxes)} released, cycle {t - ref:.1f} s"
            )
        self.grasp_t = None

    def trigger_cb(self, msg):
        if getattr(msg, "data", True):
            self.trigger_times.append(self.now())

    def timer_cb(self):
        # Subscribe to the trigger topic once its type is known
        if self.trigger_sub is None and get_message is not None:
            for name, types in self.get_topic_names_and_types():
                if name == self.trigger_topic and types:
                    try:
                        cls = get_message(types[0])
                        self.trigger_sub = self.create_subscription(
                            cls, name, self.trigger_cb, 10
                        )
                        self.get_logger().info(
                            f"Counting perception triggers on {name}"
                        )
                    except (AttributeError, ModuleNotFoundError, ValueError):
                        pass
                    break

        if self.state == self.RUNNING:
            candidates = [
                self.start_time,
                self.base.last_active,
                self.arm.last_active,
                self.last_suction_t,
            ]
            last = max(c for c in candidates if c is not None)
            if self.now() - last > self.idle_timeout:
                self.finish(last, f"idle for more than {self.idle_timeout:.0f} s")

    # ----------------------------------------------------------------------- #
    def report(self):
        t0, t1 = self.start_time, self.end_time
        total = t1 - t0
        n = len(self.boxes)

        cycles = [b["release_t"] - b["ref_t"] for b in self.boxes]
        picks = [b["grasp_t"] - b["ref_t"] for b in self.boxes]
        holds = [b["release_t"] - b["grasp_t"] for b in self.boxes]
        steady = cycles[1:]  # first cycle includes entering the container

        base_iv = self.base.intervals(t0, t1)
        arm_iv = self.arm.intervals(t0, t1)
        busy_iv = merge_intervals(base_iv + arm_iv)
        base_t = total_duration(base_iv)
        arm_t = total_duration(arm_iv)
        both_t = base_t + arm_t - total_duration(busy_iv)
        idle_t = total - total_duration(busy_iv)
        triggers = [t for t in self.trigger_times if t0 <= t <= t1]

        boxes_out = []
        for i, b in enumerate(self.boxes, start=1):
            boxes_out.append(
                {
                    "box": i,
                    "grasp_time_s": b["grasp_t"] - t0,
                    "release_time_s": b["release_t"] - t0,
                    "pick_s": b["grasp_t"] - b["ref_t"],
                    "hold_s": b["release_t"] - b["grasp_t"],
                    "cycle_s": b["release_t"] - b["ref_t"],
                    "perception_triggers": sum(
                        1 for t in triggers if b["ref_t"] < t <= b["release_t"]
                    ),
                    "base_offset_m": b["base_offset"],
                }
            )

        summary = {
            "end_reason": self.end_reason,
            "total_time_s": total,
            "boxes": n,
            "throughput_boxes_per_hour": n / total * 3600.0 if total > 0 else 0.0,
            "cycle_s": describe(cycles),
            "steady_cycle_s": describe(steady),
            "pick_s": describe(picks),
            "hold_s": describe(holds),
            "time_to_first_grasp_s": (
                self.boxes[0]["grasp_t"] - t0 if self.boxes else None
            ),
            "time_after_last_release_s": (
                t1 - self.boxes[-1]["release_t"] if self.boxes else None
            ),
            "base": {
                "distance_m": self.base_distance,
                "max_departure_m": self.max_departure,
                "moves": len(base_iv),
                "moving_time_s": base_t,
            },
            "arm": {"moving_time_s": arm_t},
            "base_and_arm_simultaneous_s": both_t,
            "idle_time_s": idle_t,
            "perception_triggers": len(triggers),
            "per_box": boxes_out,
        }

        text = self.format_report(summary)
        print(text)

        folder = os.path.join(
            self.output_dir, datetime.now().strftime("run_%Y%m%d_%H%M%S")
        )
        try:
            os.makedirs(folder, exist_ok=True)
            with open(os.path.join(folder, "report.txt"), "w") as f:
                f.write(text + "\n")
            with open(os.path.join(folder, "summary.json"), "w") as f:
                json.dump(summary, f, indent=2)
            with open(os.path.join(folder, "boxes.csv"), "w", newline="") as f:
                if boxes_out:
                    writer = csv.DictWriter(f, fieldnames=list(boxes_out[0]))
                    writer.writeheader()
                    writer.writerows(boxes_out)
            self.plot_timeline(folder, base_iv, arm_iv, t0, t1)
            self.get_logger().info(f"Report saved to {folder}")
        except OSError as e:
            self.get_logger().error(f"Could not save the report: {e}")

    @staticmethod
    def format_report(s):
        def pct(x):
            return 100.0 * x / s["total_time_s"] if s["total_time_s"] > 0 else 0.0

        def stat_line(label, d):
            if d is None:
                return f"  {label:<22} n/a"
            return (
                f"  {label:<22} mean {d['mean']:6.1f} s | median "
                f"{d['median']:6.1f} s | std {d['std']:5.1f} s | "
                f"min {d['min']:6.1f} s | max {d['max']:6.1f} s"
            )

        lines = [
            "=" * 78,
            "UNLOADING REPORT",
            "=" * 78,
            f"End condition:           {s['end_reason']}",
            f"Total time:              {mmss(s['total_time_s'])} "
            f"({s['total_time_s']:.1f} s)",
            f"Boxes unloaded:          {s['boxes']}",
            f"Throughput:              {s['throughput_boxes_per_hour']:.1f} boxes/h",
            "",
            "Per box",
            stat_line("Cycle (all boxes)", s["cycle_s"]),
            stat_line("Cycle (from box 2)", s["steady_cycle_s"]),
            stat_line("Pick (release->grasp)", s["pick_s"]),
            stat_line("Hold (grasp->release)", s["hold_s"]),
        ]
        if s["time_to_first_grasp_s"] is not None:
            lines += [
                f"  Start -> first grasp:  {s['time_to_first_grasp_s']:.1f} s",
                f"  Last release -> end:   {s['time_after_last_release_s']:.1f} s",
            ]
        if s["per_box"]:
            slowest = max(s["per_box"], key=lambda b: b["cycle_s"])
            lines.append(
                f"  Slowest box:           #{slowest['box']} "
                f"({slowest['cycle_s']:.1f} s)"
            )

        b = s["base"]
        lines += [
            "",
            "Time breakdown",
            f"  Base moving:           {b['moving_time_s']:7.1f} s "
            f"({pct(b['moving_time_s']):5.1f} %)",
            f"  Arm moving:            {s['arm']['moving_time_s']:7.1f} s "
            f"({pct(s['arm']['moving_time_s']):5.1f} %)",
            f"  Base and arm together: {s['base_and_arm_simultaneous_s']:7.1f} s "
            f"({pct(s['base_and_arm_simultaneous_s']):5.1f} %)",
            f"  Nothing moving:        {s['idle_time_s']:7.1f} s "
            f"({pct(s['idle_time_s']):5.1f} %)",
            "",
            "Base",
            f"  Distance travelled:    {b['distance_m']:.2f} m",
            f"  Max from start:        {b['max_departure_m']:.2f} m",
            f"  Separate moves:        {b['moves']}",
            "",
            f"Perception triggers:     {s['perception_triggers']}"
            + (
                f" ({s['perception_triggers'] / s['boxes']:.2f} per box)"
                if s["boxes"]
                else ""
            ),
        ]

        if s["per_box"]:
            lines += [
                "",
                f"{'Box':>4} {'Grasp@':>8} {'Release@':>9} {'Pick':>7} "
                f"{'Hold':>7} {'Cycle':>7} {'Trig':>5} {'BaseOff':>8}",
            ]
            for r in s["per_box"]:
                off = (
                    f"{r['base_offset_m']:.2f} m"
                    if r["base_offset_m"] is not None
                    else "n/a"
                )
                lines.append(
                    f"{r['box']:>4} {r['grasp_time_s']:>7.1f}s "
                    f"{r['release_time_s']:>8.1f}s {r['pick_s']:>6.1f}s "
                    f"{r['hold_s']:>6.1f}s {r['cycle_s']:>6.1f}s "
                    f"{r['perception_triggers']:>5} {off:>8}"
                )
        lines.append("=" * 78)
        return "\n".join(lines)

    def plot_timeline(self, folder, base_iv, arm_iv, t0, t1):
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn("matplotlib not found, skipping timeline")
            return

        suction_iv = [
            (max(a, t0), min(b, t1))
            for a, b in self.suction_segments
            if b > t0 and a < t1
        ]
        rows = [
            ("Base", base_iv, "tab:blue"),
            ("Arm", arm_iv, "tab:orange"),
            ("Suction", suction_iv, "tab:green"),
        ]

        fig, ax = plt.subplots(figsize=(12, 3))
        for i, (label, iv, color) in enumerate(rows):
            ax.broken_barh(
                [(a - t0, b - a) for a, b in iv],
                (len(rows) - 1 - i - 0.4, 0.8),
                color=color,
            )
        for b in self.boxes:
            ax.axvline(b["release_t"] - t0, color="grey", lw=0.6, ls="--")
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([r[0] for r in reversed(rows)])
        ax.set_xlim(0, t1 - t0)
        ax.set_xlabel("Time [s]")
        ax.set_title(
            f"Unloading timeline, {len(self.boxes)} boxes "
            f"(dashed lines are releases)"
        )
        fig.tight_layout()
        fig.savefig(os.path.join(folder, "timeline.png"), dpi=200)
        plt.close(fig)


def main():
    rclpy.init()
    node = UnloadingMetrics()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C or the launch system stopping the node: report what we have
        if node.state == node.RUNNING:
            node.finish(node.last_t, "interrupted (shutdown)")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
