"""
Check the full wiring chain from ros2_bridge's ROS2 Subscribe Twist
through to differential_controller, on turtlebot3_lidar.usd. Prints
every relevant node's actual connections so we can verify what's
present vs. missing after restoring an older backup.

Usage:
    /path/to/isaac-sim/python.sh check_full_wiring_chain.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from pxr import Usd  # noqa: E402

ROBOT_FILE = ("/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/"
              "multi_robot_lidar/isaac_sim/assets/turtlebot3_lidar.usd")

GRAPHS = "/Root/turtlebot3_burger/Graphs"
DIFF_DRIVE = f"{GRAPHS}/differential_drive/differential_drive"
ROS_GRAPH = f"{GRAPHS}/ros2_bridge/ros_graph"

NODES_TO_CHECK = {
    "ros2_subscribe_twist": f"{ROS_GRAPH}/ros2_subscribe_twist",
    "break_3_vector": f"{DIFF_DRIVE}/break_3_vector",
    "scale_to_from_stage_units": f"{DIFF_DRIVE}/scale_to_from_stage_units",
    "break_3_vector_01": f"{DIFF_DRIVE}/break_3_vector_01",
    "differential_controller": f"{DIFF_DRIVE}/differential_controller",
    "articulation_controller": f"{DIFF_DRIVE}/articulation_controller",
    "on_physics_step": f"{DIFF_DRIVE}/on_physics_step",
}

stage = Usd.Stage.Open(ROBOT_FILE)

# First check the structural containers, since a restored backup might
# not have Graphs reparented / ros2_bridge payloaded at all.
print(">>> Structural check:")
for path in ["/Root/turtlebot3_burger", GRAPHS,
             f"{GRAPHS}/differential_drive", f"{GRAPHS}/ros2_bridge",
             DIFF_DRIVE, ROS_GRAPH]:
    p = stage.GetPrimAtPath(path)
    print(f"    {path}  -->  IsValid: {p.IsValid()}")
print()

for name, path in NODES_TO_CHECK.items():
    prim = stage.GetPrimAtPath(path)
    print(f">>> {name}  ({path})")
    print(f"    IsValid: {prim.IsValid()}")
    if not prim.IsValid():
        print("    --- SKIPPING, prim not found ---")
        continue
    for attr in prim.GetAttributes():
        if attr.HasAuthoredConnections():
            targets = attr.GetConnections()
            print(f"    {attr.GetName()}  <--  {[str(t) for t in targets]}")
    print("---")

# targetPrim is a relationship, not a regular attribute connection --
# check it separately.
artic_prim = stage.GetPrimAtPath(NODES_TO_CHECK["articulation_controller"])
if artic_prim.IsValid():
    target_rel = artic_prim.GetRelationship("inputs:targetPrim")
    targets = target_rel.GetTargets() if target_rel else []
    print(f">>> articulation_controller.inputs:targetPrim  -->  {[str(t) for t in targets]}")

simulation_app.close()
