"""
Step 3b: Does the robot USD file work correctly on its own?

Opens turtlebot3_lidar.usd directly as the main stage (no referencing, no
environment) to check whether Geometry/Physics/Materials/Graphs are all
intact when the file is loaded by itself. If they're missing even here,
the problem is inside the saved file (likely broken relative asset paths
from when it was saved), not caused by referencing it elsewhere.

Usage:
    /path/to/isaac-sim/python.sh step3b_open_robot_alone_test.py \
        --robot /path/to/turtlebot3_lidar.usd
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--robot", type=str, required=True,
                     help="Path to the saved robot asset USD.")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

print(">>> Creating SimulationApp...")
simulation_app = SimulationApp({"headless": args.headless})
print(">>> SimulationApp created.")

import omni.usd  # noqa: E402

print(f">>> Opening robot USD directly as the stage: {args.robot}")
success = omni.usd.get_context().open_stage(args.robot)
print(f">>> open_stage returned: {success}")

stage = omni.usd.get_context().get_stage()
print(f">>> Default prim: {stage.GetDefaultPrim()}")

print(">>> Full prim hierarchy:")
for prim in stage.Traverse():
    print(f"    {prim.GetPath()}  ({prim.GetTypeName()})")

print(">>> Keeping app open. Look at the viewport -- does the robot appear")
print(">>> fully intact (chassis, wheels, lidar) with no red/broken icons")
print(">>> in the Stage panel? Close the window (or Ctrl+C) when done.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
