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
