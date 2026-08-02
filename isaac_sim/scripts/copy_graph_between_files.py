"""
Final wiring step: connect differential_drive's two dangling inputs
(left disconnected when ros2_context/ros2_subscribe_twist were deleted
from that file) to ros2_bridge's ROS2 Subscribe Twist outputs -- linking
the two separately-payloaded graphs together on the composed robot.

IMPORTANT: if you renamed "ros_graph" to something else (e.g.
"ros_drive"), update ROS_GRAPH_NAME below to match before running.

Usage:
    /path/to/isaac-sim/python.sh wire_ros2_bridge_to_drive.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from pxr import Usd, Sdf  # noqa: E402

ROBOT_FILE = ("/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/"
              "multi_robot_lidar/isaac_sim/assets/turtlebot3_lidar.usd")

# Change this if you renamed "ros_graph" to something else.
ROS_GRAPH_NAME = "ros_graph"

ROBOT_ROOT = "/Root/turtlebot3_burger"
GRAPHS = f"{ROBOT_ROOT}/Graphs"
DIFF_DRIVE_GRAPH = f"{GRAPHS}/differential_drive/differential_drive"
ROS_SUBSCRIBE_TWIST = f"{GRAPHS}/ros2_bridge/{ROS_GRAPH_NAME}/ros2_subscribe_twist"

BREAK_3_VECTOR = f"{DIFF_DRIVE_GRAPH}/break_3_vector"
SCALE_NODE = f"{DIFF_DRIVE_GRAPH}/scale_to_from_stage_units"

stage = Usd.Stage.Open(ROBOT_FILE)

full_text = stage.GetRootLayer().ExportToString()
idx = full_text.find('"Graphs"')
if idx != -1:
    print(">>> Raw text around 'Graphs':")
    print(full_text[max(0, idx - 200):idx + 1500])
else:
    print(">>> 'Graphs' not found anywhere in the raw text!")
print("---")

# Check every level of the path chain to find exactly where it breaks.
levels_to_check = [
    "/Root",
    ROBOT_ROOT,
    GRAPHS,
    f"{GRAPHS}/differential_drive",
    DIFF_DRIVE_GRAPH,
    f"{GRAPHS}/ros2_bridge",
    f"{GRAPHS}/ros2_bridge/{ROS_GRAPH_NAME}",
]
print(">>> Checking every level of the path chain:")
for level in levels_to_check:
    p = stage.GetPrimAtPath(level)
    print(f"    {level}  -->  IsValid: {p.IsValid()}")

# Sanity check both endpoints exist before wiring.
subscribe_prim = stage.GetPrimAtPath(ROS_SUBSCRIBE_TWIST)
break_prim = stage.GetPrimAtPath(BREAK_3_VECTOR)
scale_prim = stage.GetPrimAtPath(SCALE_NODE)

print(f">>> ros2_subscribe_twist valid: {subscribe_prim.IsValid()} ({ROS_SUBSCRIBE_TWIST})")
print(f">>> break_3_vector valid: {break_prim.IsValid()} ({BREAK_3_VECTOR})")
print(f">>> scale_to_from_stage_units valid: {scale_prim.IsValid()} ({SCALE_NODE})")

if not (subscribe_prim.IsValid() and break_prim.IsValid() and scale_prim.IsValid()):
    print(">>> One or more paths invalid -- stopping. Check ROS_GRAPH_NAME and")
    print(">>> the printed paths above against your actual Stage panel structure.")
else:
    # over "differential_drive" { over "break_3_vector" { inputs:tuple.connect = ... } }
    tuple_attr = break_prim.GetAttribute("inputs:tuple")
    if not tuple_attr:
        tuple_attr = break_prim.CreateAttribute("inputs:tuple", Sdf.ValueTypeNames.Token)
    tuple_attr.SetConnections([f"{ROS_SUBSCRIBE_TWIST}.outputs:angularVelocity"])
    print(f">>> Connected {BREAK_3_VECTOR}.inputs:tuple -> "
          f"{ROS_SUBSCRIBE_TWIST}.outputs:angularVelocity")

    value_attr = scale_prim.GetAttribute("inputs:value")
    if not value_attr:
        value_attr = scale_prim.CreateAttribute("inputs:value", Sdf.ValueTypeNames.Token)
    value_attr.SetConnections([f"{ROS_SUBSCRIBE_TWIST}.outputs:linearVelocity"])
    print(f">>> Connected {SCALE_NODE}.inputs:value -> "
          f"{ROS_SUBSCRIBE_TWIST}.outputs:linearVelocity")

    stage.GetRootLayer().Save()
    print(">>> Saved.")

simulation_app.close()
