"""
Apply the base CollisionAPI to both tires -- MeshCollisionAPI alone
(which we already set to convexHull/SDF Mesh) only configures HOW a
mesh is approximated IF it's a collider; CollisionAPI is what actually
makes it a real physical collision shape. Without it, wheels spin with
zero ground contact -- exactly the "spins but doesn't move" symptom.

Usage:
    /path/to/isaac-sim/python.sh fix_tire_collision.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from pxr import Usd, UsdPhysics  # noqa: E402

ROBOT_FILE = ("/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/"
              "multi_robot_lidar/isaac_sim/assets/turtlebot3_lidar.usd")

BASE = "/Root/turtlebot3_burger/Geometry/base_footprint/base_link"
TIRES = [
    f"{BASE}/wheel_left_link/left_tire",
    f"{BASE}/wheel_right_link/right_tire",
]

stage = Usd.Stage.Open(ROBOT_FILE)

for tire_path in TIRES:
    prim = stage.GetPrimAtPath(tire_path)
    print(f">>> {tire_path}  IsValid: {prim.IsValid()}")
    if not prim.IsValid():
        continue
    print(f"    Has CollisionAPI before: {prim.HasAPI(UsdPhysics.CollisionAPI)}")
    UsdPhysics.CollisionAPI.Apply(prim)
    print(f"    Has CollisionAPI after: {prim.HasAPI(UsdPhysics.CollisionAPI)}")

stage.GetRootLayer().Save()
print(">>> Saved.")

simulation_app.close()
