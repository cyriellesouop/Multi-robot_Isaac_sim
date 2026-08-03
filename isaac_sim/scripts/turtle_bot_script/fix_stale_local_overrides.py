"""
Rewire ros2_subscribe_twist's execIn from on_playback_tick to
on_physics_step -- diagnostic test to see if unifying everything onto
one clock changes anything. Leaves on_playback_tick in place, unused,
rather than trying to delete it graphically.

Usage:
    /path/to/isaac-sim/python.sh rewire_ros2_to_physics_step.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("isaacsim.ros2.bridge")

for _ in range(10):
    simulation_app.update()

import omni.usd  # noqa: E402

DIFF_DRIVE_FILE = ("/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/"
                    "multi_robot_lidar/isaac_sim/assets/differential_drive.usd")
GRAPH_PATH = "/Root/differential_drive"

omni.usd.get_context().open_stage(DIFF_DRIVE_FILE)
stage = omni.usd.get_context().get_stage()

exec_attr = stage.GetPrimAtPath(f"{GRAPH_PATH}/ros2_subscribe_twist").GetAttribute("inputs:execIn")
print(f">>> Before: {[str(c) for c in exec_attr.GetConnections()]}")
exec_attr.SetConnections([f"{GRAPH_PATH}/on_physics_step.outputs:step"])
print(f">>> After:  {[str(c) for c in exec_attr.GetConnections()]}")

for _ in range(10):
    simulation_app.update()

stage.GetRootLayer().Save()
print(">>> Saved.")

simulation_app.close()
