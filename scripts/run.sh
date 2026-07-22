#!/usr/bin/env bash
#
# Build (if needed) and run the fleet_cmd_vel_publisher ROS2 package.
#
# Usage:
#   ./run.sh --num-robots 4
#   ./run.sh --num-robots 4 --commands 0.2,0.0 0.2,0.0 0.0,0.5 0.0,-0.5
#   ./run.sh --num-robots 4 --duration 10
#
# Any arguments you pass to this script are forwarded as-is to
# `ros2 run fleet_cmd_vel_piublisher publish_fleet_cmd_vel`.
#
# Assumes this script sits in the same directory as the
# fleet_cmd_vel_publisher/ package folder (package.xml, CMakeLists.txt, src/).

set -e  # stop immediately if any command fails

# --- Config ---------------------------------------------------------------
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
#WORKSPACE_DIR="./multiRobot_ws"

PACKAGE_FOLDER_NAME="fleet_cmd_vel_publisher"        # folder name on disk under src/
PACKAGE_NAME="fleet_cmd_vel_publisher"  # internal name, from package.xml <name>
EXECUTABLE_NAME="publish_fleet_cmd_vel"

# Directory this script lives in (project_root/scripts), so it works
# regardless of your current working directory when you invoke it.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The workspace lives one level up from scripts/, at project_root/multiRobot_ws.
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../multiRobot_ws" && pwd)"

#PACKAGE_SOURCE_DIR="$SCRIPT_DIR/$PACKAGE_NAME"

# --- 1. Source the base ROS2 install ---------------------------------------
if [[ -z "$ROS_DISTRO" ]]; then
    echo "Sourcing base ROS2 install: $ROS_DISTRO_SETUP"
    source "$ROS_DISTRO_SETUP"
else
    echo "ROS2 already sourced (ROS_DISTRO=$ROS_DISTRO), skipping."
fi


# --- 2. Sanity check the package is where we expect it ----------------------
if [[ ! -d "$WORKSPACE_DIR/$PACKAGE_FOLDER_NAME/src" ]]; then
    echo "ERROR: $WORKSPACE_DIR/$PACKAGE_FOLDER_NAME/src not found."
    echo "Make sure your package folder lives directly under multiRobot_ws/src/."
    exit 1
fi

export PATH="/usr/bin:$PATH"
echo "DEBUG: cmake resolves to -> $(which cmake)"
echo "DEBUG: PATH is -> $PATH"

# --- 2. Create the workspace if it doesn't exist yet -----------------------
#if [[ ! -d "$WORKSPACE_DIR/src" ]]; then
 #   echo "Creating workspace at $WORKSPACE_DIR"
  #  mkdir -p "$WORKSPACE_DIR/src"
#fi

# --- 3. Copy/update the package into the workspace --------------------------
#echo "Copying $PACKAGE_NAME into workspace"
#rm -rf "$WORKSPACE_DIR/src/$PACKAGE_NAME"
#cp -r "$PACKAGE_SOURCE_DIR" "$WORKSPACE_DIR/src/"

# --- 4. Build ----------------------------------------------------------------
echo "Building $PACKAGE_NAME"
cd "$WORKSPACE_DIR"
colcon build --packages-select "$PACKAGE_NAME"

# --- 5. Source the workspace overlay -----------------------------------------
echo "Sourcing workspace overlay"
source "$WORKSPACE_DIR/install/setup.bash"

# --- 6. Run, forwarding any arguments given to this script -------------------
echo "Running $EXECUTABLE_NAME $*"
ros2 run "$PACKAGE_NAME" "$EXECUTABLE_NAME" "$@"
