#!/usr/bin/env python3
"""
analyze_trial.py

Post-hoc analysis of a single trial:
  1. Reads a recorded rosbag2 and computes:
       - RTF (Real-Time Factor) from /clock messages
       - Path length (odometry-integrated distance) from /chassis/odom
  2. Parses the isaac_ros_navigation_goal terminal log and extracts:
       - number of goals sent
       - number of results received (proxy for goals that returned, pending
         the success/failure verification test discussed separately)
       - per-goal duration (wall-clock, from log timestamps)
  3. Writes one summary row (appended) to a master results CSV.

Usage:
    python3 analyze_trial.py \
        --bag trial_R1_S-low_rep1_bag \
        --navlog trial_R1_S-low_rep1_navgoal.log \
        --robots 1 --tier low --rep 1 \
        --summary results_summary.csv

Requires: rosbag2_py, rclpy (available once your ROS 2 workspace is sourced).
Run this in a terminal where you've already sourced ROS 2 + your workspace,
same as any other ros2 command.
"""

import argparse
import csv
import os
import re
import sys
from pathlib import Path


# ---------------------------------------------------------------------------
# Part 1: rosbag2 analysis (RTF + path length)
# ---------------------------------------------------------------------------

def analyze_bag(bag_path: str):
    """
    Reads /clock and /chassis/odom from the bag and returns:
        rtf: float or None
        path_length_m: float or None
        n_clock_msgs: int
        n_odom_msgs: int
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosgraph_msgs.msg import Clock
        from nav_msgs.msg import Odometry
    except ImportError as e:
        print(f"ERROR: could not import ROS 2 bag/message libraries ({e}).")
        print("Make sure you've sourced ROS 2 and your workspace in this terminal.")
        sys.exit(1)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if "/clock" not in type_map:
        print("WARNING: /clock not found in bag — cannot compute RTF.")
    if "/chassis/odom" not in type_map:
        print("WARNING: /chassis/odom not found in bag — cannot compute path length.")

    clock_records = []   # (bag_wall_time_ns, sim_time_sec)
    odom_positions = []  # (bag_wall_time_ns, x, y)

    while reader.has_next():
        topic, data, bag_wall_time_ns = reader.read_next()

        if topic == "/clock":
            msg = deserialize_message(data, Clock)
            sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
            clock_records.append((bag_wall_time_ns, sim_time_sec))

        elif topic == "/chassis/odom":
            msg = deserialize_message(data, Odometry)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            odom_positions.append((bag_wall_time_ns, x, y))

    # --- RTF: (delta sim time) / (delta wall time) across the recording ---
    rtf = None
    delta_sim_s = None
    delta_wall_s = None
    if len(clock_records) >= 2:
        first_wall_ns, first_sim = clock_records[0]
        last_wall_ns, last_sim = clock_records[-1]
        delta_wall_s = (last_wall_ns - first_wall_ns) * 1e-9
        delta_sim_s = last_sim - first_sim
        if delta_wall_s > 0:
            rtf = delta_sim_s / delta_wall_s

    # --- Path length: sum of consecutive Euclidean distances ---
    path_length_m = None
    if len(odom_positions) >= 2:
        total = 0.0
        for i in range(1, len(odom_positions)):
            _, x0, y0 = odom_positions[i - 1]
            _, x1, y1 = odom_positions[i]
            total += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        path_length_m = total

    return {
        "rtf": rtf,
        "sim_time_elapsed_s": delta_sim_s,
        "wall_time_elapsed_s": delta_wall_s,
        "path_length_m": path_length_m,
        "n_clock_msgs": len(clock_records),
        "n_odom_msgs": len(odom_positions),
    }


# ---------------------------------------------------------------------------
# Part 2: navigation goal log parsing
# ---------------------------------------------------------------------------

# Matches lines like:
# [SetNavigationGoal-1] [INFO] [1785552951.839576014] [set_navigation_goal]: Sending first goal
# [SetNavigationGoal-1] [INFO] [1785552951.840227982] [set_navigation_goal]: Generated goal pose: [...]
# [SetNavigationGoal-1] [INFO] [1785553030.601953323] [set_navigation_goal]: Result: ...
LOG_LINE_RE = re.compile(
    r"\[SetNavigationGoal-\d+\]\s*\[INFO\]\s*\[(?P<ts>\d+\.\d+)\]\s*\[set_navigation_goal\]:\s*(?P<msg>.*)"
)


def parse_navgoal_log(log_path: str):
    """
    Returns:
        n_goals_generated: int
        n_results: int
        goal_durations_s: list of float (time between a goal's "Goal accepted"
                                          and the following "Result" line)
        completed_cleanly: bool (whether the process printed the clean-exit line)
    """
    events = []  # (timestamp_float, event_type)

    with open(log_path, "r", errors="ignore") as f:
        for line in f:
            m = LOG_LINE_RE.search(line)
            if not m:
                continue
            ts = float(m.group("ts"))
            msg = m.group("msg")

            if "Goal accepted" in msg:
                events.append((ts, "accepted"))
            elif msg.startswith("Result:"):
                events.append((ts, "result"))
            elif "Generated goal pose" in msg:
                events.append((ts, "generated"))

    n_goals_generated = sum(1 for _, e in events if e == "generated")
    n_results = sum(1 for _, e in events if e == "result")

    # Pair each "accepted" with the next "result" to get per-goal duration.
    durations = []
    pending_accept_ts = None
    for ts, etype in events:
        if etype == "accepted":
            pending_accept_ts = ts
        elif etype == "result" and pending_accept_ts is not None:
            durations.append(ts - pending_accept_ts)
            pending_accept_ts = None

    with open(log_path, "r", errors="ignore") as f:
        content = f.read()
    completed_cleanly = "process has finished cleanly" in content

    return {
        "n_goals_generated": n_goals_generated,
        "n_results": n_results,
        "goal_durations_s": durations,
        "completed_cleanly": completed_cleanly,
    }


# ---------------------------------------------------------------------------
# Part 3: Nav2-side log parsing (true success/failure)
# ---------------------------------------------------------------------------

# The isaac_ros_navigation_goal log always prints "Result: ...Empty()" for a
# goal regardless of whether it succeeded or failed — the NavigateToPose
# action's result payload is empty by design, so it carries no outcome info.
# True success/failure only shows up in Nav2's own bt_navigator log, e.g.:
#   [bt_navigator]: Goal succeeded
#   [ERROR] [bt_navigator]: Goal failed
# We match on the bt_navigator tag plus "Goal succeeded"/"Goal failed" text,
# tolerant of the exact log level tag and node-container prefix ROS 2 adds.
NAV2_OUTCOME_RE = re.compile(
    r"\[bt_navigator\]:\s*(?P<outcome>Goal succeeded|Goal failed)"
)


def parse_nav2_log(log_path: str):
    """
    Returns:
        n_succeeded: int
        n_failed: int
        outcomes: list of ("succeeded"|"failed", line_index) in the order
                  they appeared, useful for lining up against the navgoal
                  log's goal order if needed.
    """
    outcomes = []

    with open(log_path, "r", errors="ignore") as f:
        for i, line in enumerate(f):
            m = NAV2_OUTCOME_RE.search(line)
            if not m:
                continue
            outcome = "succeeded" if m.group("outcome") == "Goal succeeded" else "failed"
            outcomes.append((outcome, i))

    n_succeeded = sum(1 for o, _ in outcomes if o == "succeeded")
    n_failed = sum(1 for o, _ in outcomes if o == "failed")

    return {
        "n_succeeded": n_succeeded,
        "n_failed": n_failed,
        "outcomes": outcomes,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Analyze one trial (bag + nav goal log).")
    parser.add_argument("--bag", required=True, help="Path to the rosbag2 directory.")
    parser.add_argument("--navlog", required=True, help="Path to the nav goal terminal log file.")
    parser.add_argument("--nav2log", required=True, help="Path to the carter_navigation (Nav2) terminal log file, used to determine true goal success/failure.")
    parser.add_argument("--robots", required=True, type=int, help="Fleet size (R).")
    parser.add_argument("--tier", required=True, help="Sensor tier (low/medium/high).")
    parser.add_argument("--rep", required=True, type=int, help="Repetition number.")
    parser.add_argument("--summary", default="results_summary.csv", help="Master summary CSV to append to.")
    args = parser.parse_args()

    if not Path(args.bag).exists():
        print(f"ERROR: bag path not found: {args.bag}")
        sys.exit(1)
    if not Path(args.navlog).exists():
        print(f"ERROR: nav log not found: {args.navlog}")
        sys.exit(1)
    if not Path(args.nav2log).exists():
        print(f"ERROR: nav2 log not found: {args.nav2log}")
        sys.exit(1)

    print(f"Analyzing bag: {args.bag}")
    bag_results = analyze_bag(args.bag)

    print(f"Analyzing nav goal log: {args.navlog}")
    nav_results = parse_navgoal_log(args.navlog)

    print(f"Analyzing nav2 log: {args.nav2log}")
    nav2_results = parse_nav2_log(args.nav2log)

    n_outcomes = nav2_results["n_succeeded"] + nav2_results["n_failed"]
    success_rate = (
        nav2_results["n_succeeded"] / n_outcomes if n_outcomes > 0 else None
    )

    avg_goal_duration = (
        sum(nav_results["goal_durations_s"]) / len(nav_results["goal_durations_s"])
        if nav_results["goal_durations_s"] else None
    )

    row = {
        "robots": args.robots,
        "sensor_tier": args.tier,
        "rep": args.rep,
        "rtf": bag_results["rtf"],
        "sim_time_elapsed_s": bag_results["sim_time_elapsed_s"],
        "wall_time_elapsed_s": bag_results["wall_time_elapsed_s"],
        "path_length_m": bag_results["path_length_m"],
        "n_clock_msgs": bag_results["n_clock_msgs"],
        "n_odom_msgs": bag_results["n_odom_msgs"],
        "n_goals_generated": nav_results["n_goals_generated"],
        "n_results": nav_results["n_results"],
        "n_succeeded": nav2_results["n_succeeded"],
        "n_failed": nav2_results["n_failed"],
        "success_rate": success_rate,
        "avg_goal_duration_s": avg_goal_duration,
        "goal_durations_s": ";".join(f"{d:.2f}" for d in nav_results["goal_durations_s"]),
        "completed_cleanly": nav_results["completed_cleanly"],
    }

    write_header = not Path(args.summary).exists()
    with open(args.summary, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)

    print("\n--- Summary row written ---")
    for k, v in row.items():
        print(f"  {k}: {v}")
    print(f"\nAppended to: {args.summary}")


if __name__ == "__main__":
    main()
