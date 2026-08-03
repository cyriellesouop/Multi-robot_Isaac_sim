#!/usr/bin/env bash
#
# Launch the fleet spawner script through Isaac Sim's own Python
# interpreter (isaacsim is only importable inside it, not system python3).
#
# Usage:
#   ./spawn.sh --num-robots 4
#   ./spawn.sh --num-robots 4 --headless
#   ./spawn.sh --num-robots 4 --env /other/env.usd --robot /other/robot.usd
 
set -e
 
ISAAC_PYTHON="/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/python.sh"

# Isaac Sim bundles its own Python (3.12), but ROS2 Humble's system rclpy
# is compiled for Python 3.10 -- neither can load as-is, which otherwise
# only shows as a warning but then SEGFAULTS once the OmniGraph actually
# starts ticking ROS2 nodes (ros2_context, ros2_subscribe_twist, etc.).
# These exports point rclpy at Isaac Sim's own bundled, matching-Python
# ROS2 libraries instead, so it loads correctly. Must be set before
# python.sh launches -- this is an OS-level library search path, not
# something a fix inside the Python script itself can address.
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
 
# rclpy depends on two separate things, found in two separate locations,
# resolved by two separate mechanisms:
#   1. The Python package itself (rclpy/, with its _rclpy_pybind11
#      C extension for Isaac Sim's bundled Python 3.12) -- needs
#      PYTHONPATH, Python's own module search path.
#   2. Underlying C libraries (librcl, the RMW/DDS implementation, etc.)
#      that the C extension links against at runtime -- needs
#      LD_LIBRARY_PATH, the OS's shared-library search path.
ISAACSIM_ROS2_HUMBLE_DIR="/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/exts/isaacsim.ros2.core/humble"
export PYTHONPATH="$ISAACSIM_ROS2_HUMBLE_DIR/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAACSIM_ROS2_HUMBLE_DIR/lib"
 
#echo "DEBUG: ROS_DISTRO=$ROS_DISTRO"
#echo "DEBUG: RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
#echo "DEBUG: PYTHONPATH=$PYTHONPATH"
#echo "DEBUG: LD_LIBRARY_PATH=$LD_LIBRARY_PATH"

#/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/python.sh open_and_edit_usd.py \
#  --robot /home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/multi_robot_lidar/isaac_sim/assets/turtlebot3_lidar.usd
 
# Directory this script lives in, so it works regardless of your current
# working directory when you invoke it.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPAWN_SCRIPT="$SCRIPT_DIR/script_spawn_multi_robot_test.py"
 
# Project asset paths, hardcoded here so you don't have to type them every
# run. "$@" is appended after these, so any --env/--robot you pass on the
# command line will override these defaults (argparse keeps the last value
# it sees for a given flag).
DEFAULT_ENV="$SCRIPT_DIR/../assets/SimpleRoom_office.usd"
DEFAULT_ROBOT="$SCRIPT_DIR/../assets/turtlebot3_lidar.usd"
 
#"$ISAAC_PYTHON" "$SPAWN_SCRIPT" --env "$DEFAULT_ENV" --robot "$DEFAULT_ROBOT" "$@"

"$ISAAC_PYTHON" "$SPAWN_SCRIPT" --env "$DEFAULT_ENV"  --robot "$DEFAULT_ROBOT" "$@"

#"$ISAAC_PYTHON" "$SPAWN_SCRIPT" --env "$DEFAULT_ENV"  --robot "$DEFAULT_ROBOT" --output "$DEFAULT_OUTPUT" "$@"
 
 
