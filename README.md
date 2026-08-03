# Multi-Robot Isaac Sim + ROS2 Fleet

Multi-robot TurtleBot3 fleet simulated in NVIDIA Isaac Sim and controlled
over ROS2, built to empirically measure ROS2 middleware bottlenecks
(DDS/bridge overhead) as fleet size grows in a GPU-accelerated simulator.

## Hierarchy and what each part implements

```
.
├── isaac_sim/
│   ├── assets/
│   │   ├── SimpleRoom_office.usd
│   │   └── turtlebot3_lidar.usd
│   └── scripts/
│       └── script_spawn_multi_robot.py
├── multiRobot_ws/
│   └── fleet_cmd_vel_publisher/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src/
│           └── publish_fleet_cmd_vel.cpp
├── readme.md
├── results/
└── scripts/
    └── run.sh
```

### `isaac_sim/assets/SimpleRoom_office.usd`
The simulation environment. A saved USD stage containing the room/world
geometry and `PhysicsScene` — built once graphically in Isaac Sim, loaded
fresh by the spawner script rather than rebuilt each run.

### `isaac_sim/assets/turtlebot3_lidar.usd`
The robot template. A saved, self-contained USD asset holding one
TurtleBot3's chassis, physics, materials, and its `Graphs` group (the
`differential_drive` OmniGraph, which bridges ROS2 `/cmd_vel` Twist
messages into wheel motion, plus the lidar publisher). Built once
graphically, then referenced multiple times rather than reconfigured per
robot.

### `isaac_sim/scripts/script_spawn_multi_robot.py`
The fleet orchestrator, run inside Isaac Sim (Script Editor or standalone).
Loads `SimpleRoom_office.usd`, then references `turtlebot3_lidar.usd` N
times into the stage at distinct positions. Sets an `isaac:namespace`
attribute (e.g. `robot_0`, `robot_1`, ...) on each robot's root prim, which
every ROS2 OmniGraph node nested inside it inherits automatically — so
each robot's `/cmd_vel`, `/scan`, etc. end up on unique topics without any
manual per-robot OmniGraph editing.

### `multiRobot_ws/fleet_cmd_vel_publisher/`
A ROS2 (`ament_cmake`) package — the workspace's build unit.

- **`package.xml`** — the package manifest: declares the package's
  internal name (`fleet_cmd_vel_publisher`), its dependencies (`rclcpp`,
  `geometry_msgs`), and that it builds via `ament_cmake`.
- **`CMakeLists.txt`** — the build instructions: locates `rclcpp` and
  `geometry_msgs`, compiles `src/publish_fleet_cmd_vel.cpp` into an
  executable named `publish_fleet_cmd_vel`, and installs it so
  `ros2 run` can find it.
- **`src/publish_fleet_cmd_vel.cpp`** — the actual node. A C++/`rclcpp`
  program that creates one Twist publisher per robot, targeting each
  robot's namespaced `/cmd_vel` topic, and publishes commands on a timer
  — equivalent to running `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist
  ...` by hand, but for N robots at once, with independent per-robot
  linear/angular commands (useful for making robots' paths cross for
  congestion/stress testing).

### `scripts/run.sh`
Build-and-run wrapper for the ROS2 package. Sources the base ROS2 install
if not already sourced, sanity-checks the package folder is where
expected, runs `colcon build --packages-select` on it, sources the
workspace's build overlay, then runs the node — forwarding whatever
arguments you pass (`--num-robots`, `--commands`, `--duration`) straight
through to the executable.

### `results/`
Destination for generated data — CSV logs, plots — from experiment runs
(CPU/GPU/DDS-latency/navigation metrics across fleet-size trials). Empty
until experiments produce output.

### `readme.md`
This file.

## Typical workflow

1. Open Isaac Sim, run `isaac_sim/scripts/script_spawn_multi_robot.py`
   (pointed at the two `isaac_sim/assets/` files) to spawn the fleet, each
   robot auto-namespaced. Hit Play.
2. From the project root, run:
   ```bash
   ./scripts/run.sh --num-robots 4
   ```
   to build (first run only, or after editing the `.cpp`) and launch the
   cmd_vel publisher, driving all spawned robots.
3. Verify in a ROS2-sourced terminal with `ros2 topic list` — each robot's
   topics should appear cleanly namespaced with no cross-talk.
   
   
   
## Nova Carter Pivot: Middleware Bottleneck Benchmarking

The project initially targeted a TurtleBot3 fleet (see the original
hierarchy above, and `isaac_sim/scripts/turtle_bot_script/`). After
significant time spent on manual OmniGraph wiring and TurtleBot3 asset
debugging, the project pivoted to **NVIDIA Nova Carter** as the fixed
robot platform, using Isaac Sim's pre-built Nav2 integration rather than
a hand-assembled fleet spawner. This section documents that pivot: the
new assets, the sensor-load control methodology, the goal design, and the
data collection pipeline built around it.

### Why Nova Carter, and why the pivot happened

TurtleBot3 required manually wiring OmniGraph nodes (differential drive,
lidar publishing, TF) per robot instance, which consumed the project's
original data-collection window (see `isaac_sim/scripts/turtle_bot_script/`
for the surviving debugging scripts —
`inspect_differential_drive.py`, `dump_differential_drive_raw.py`,
`fix_stale_local_overrides.py`, `check_full_wiring_chain.py` — kept for
reference, not part of the active pipeline). Nova Carter ships with a
complete, pre-wired ROS 2 bridge (OmniGraph action graphs for lidar,
IMU, stereo/fisheye cameras, differential drive, and TF/odometry
publishing) and an existing Nav2 integration
(`carter_navigation` package, part of the Isaac Sim ROS workspace), which
removed the asset-wiring bottleneck entirely and let the project focus on
the actual research question: middleware scaling, not asset construction.

### Scene assets

- `isaac_sim/Nova_carter_robot_usd/Nav2_Nova_Carter_robot_warehouse.usd`
  (mirrored at `isaac_sim/assets/Nav2_Nova_Carter_robot_warehouse.usd`) —
  single Nova Carter robot in the warehouse scene, used for initial
  pipeline verification and the R=1 baseline trials.
- `isaac_sim/Nova_carter_robot_usd/Nav2_Nova_Nav2_all_sensors.usd`
  (mirrored at `isaac_sim/assets/Nav2_Nova_Nav2_all_sensors.usd`) — Nova
  Carter variant with all sensor OmniGraph publishers present, used as
  the base scene for sensor-tier experiments (see below).
- `isaac_sim/assets/carter_warehouse_navigation.png` +
  `isaac_sim/assets/carter_warehouse_navigation.yaml` — the static
  occupancy map (480×776 px, 0.05 m/px, origin `[-11.975, -17.975, 0.0]`)
  used by Nav2's `map_server` for localization and global planning.
  Generated once per environment; not regenerated between trials.

> **Note:** file names for dedicated Low/Medium/High sensor-tier scene
> variants aren't yet reflected in the folder listing above — if saved
> separately (per the "save as three USD variants" recommendation),
> add their paths here (e.g. `office_scene_low_sensors.usd`,
> `_medium_`, `_high_`) once created.

### Sensor-load tiers (independent variable: S)

Sensor load is controlled by deactivating **ROS 2 OmniGraph publisher
nodes only**, not the underlying physical sensor prims — this holds GPU
rendering cost constant across tiers and isolates DDS/middleware
publishing load as the sole variable. See `isaac_sim/Troubleshooting.md`
for the full reasoning (physical-sensor-vs-publisher confounding
argument) if not already merged into this readme.

- **Low:** 2D lidar only (`ros_lidars` graph's
  `publish_front_2d_lidar_scan` / `publish_back_2d_lidar_scan`
  sub-nodes) + `transform_tree_odometry` (required at every tier) +
  `differential_drive` (actuator graph, always active).
- **Medium:** Low, plus `publish_front_3d_lidar_scan` (XT32 point cloud,
  a sub-node within `ros_lidars` — deactivated independently of its
  sibling 2D publishers) and `chassis_imu`.
- **High:** Medium, plus `front_hawk` (stereo camera OmniGraph node).

All physical sensor prims (`front_RPLidar`, `XT_32`, `front_hawk`,
etc., under `chassis_link/sensors/`) remain active at every tier.

### Goal design (fixed goals, `GoalReader`)

Goals were switched from `RandomGoalGenerator` to `GoalReader` with a
fixed goal list, to prevent goal-difficulty variance from confounding
the robot-count / sensor-tier comparison. Three goals were selected
directly off the occupancy map using
`isaac_sim/scripts/pick_goals.py` (click-to-world-coordinate tool,
validated against the map YAML's resolution/origin), representing three
distinct navigation challenges:

| Goal type | World (x, y) | Approx. distance from spawn (-6.4, -1.04) |
|---|---|---|
| Short-range | (-5.825, -2.775) | ~1.8 m |
| Long-traverse | (-7.425, 13.175) | ~14.3 m |
| Obstacle-avoidance | (1.675, -2.875) | ~8.3 m |

Configured via the `isaac_ros_navigation_goal` launch file
(`goal_generator_type: "GoalReader"`, `iteration_count: 3` — one
iteration per goal type above, not a repetition count). Repetitions
across trials (`rep1`, `rep2`, ...) are separate re-runs of this same
fixed 3-goal sequence.

### Data collection pipeline

Per trial, four things run concurrently and are saved under
`isaac_sim/scripts/nova_carter_data/trial_R{n}_S-{tier}_rep{k}_bag/`:

1. **`ros2 bag record`** — captures `/tf`, `/scan`, `/clock`,
   `/chassis/odom` (plus sensor topics matching the active tier) into
   the trial's `_bag/` folder (`metadata.yaml` + `.db3`).
2. **`isaac_ros_navigation_goal` launch output**, redirected via `tee`
   to `trial_R{n}_S-{tier}_rep{k}_navgoal.log` — goal-sent/result
   timestamps (does **not** reliably indicate success vs. failure; see
   below).
3. **`carter_navigation` (Nav2) launch output**, redirected via `tee`
   to `trial_R{n}_S-{tier}_rep{k}_nav2.log` — the authoritative source
   for goal outcome (`[bt_navigator]: Goal succeeded` /
   `Goal failed`).
4. **`isaac_sim/scripts/resource_sampler.sh`** — 1 Hz CPU/RAM/GPU/VRAM
   polling, saved as `resources.csv` per trial.

`isaac_sim/scripts/analyze_trial.py` post-processes all of the above
into one row per trial in `results_summary.csv`:
RTF (from `/clock`), sim/wall elapsed time, path length (integrated from
`/chassis/odom`), goal-generated/result counts (navgoal log), true
succeeded/failed counts and success rate (Nav2 log, cross-referenced
against the navgoal log), and per-goal durations.

> **Known finding, documented for future reference:** the
> `isaac_ros_navigation_goal` package logs an identically-formatted
> `Result: ...Empty()` line regardless of whether a goal ultimately
> succeeded or failed (the `NavigateToPose` action's result payload is
> empty by design). True task success must be read from the Nav2
> (`carter_navigation`) log, not the navgoal log alone — hence step 3
> above is required, not optional.

### Baseline capture

`isaac_sim/assets/idle_baseline.md` records system resource state at
three checkpoints per condition (idle / scene-loaded-paused /
playing-no-goals) prior to each trial's actual navigation window, used
as the reference point for attributing CPU/GPU cost to Isaac Sim's
rendering overhead vs. actual trial execution.

### Known environment quirks (see `isaac_sim/Troubleshooting.md` for full detail)

- `ROS_DOMAIN_ID` must be set explicitly (`42` in this project) to avoid
  cross-talk with other DDS participants on the shared lab network.
- `cmake` and `python3` can both be silently shadowed by conda/Xilinx
  Vitis `PATH` entries; use `export PATH=/usr/bin:$PATH` or
  `conda deactivate` before `colcon build` or running `analyze_trial.py`.
- `/scan` publishes at ~2.8 Hz (confirmed via live `ros2 topic hz`
  cross-check against bag-recorded message counts) — this is Nova
  Carter's actual configured 2D lidar rate, not a QoS-drop artifact,
  despite its best-effort reliability QoS profile.
   
   
