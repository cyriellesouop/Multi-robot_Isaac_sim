"""
Step 3c: What is actually composing (or failing to compose) the robot prim?

Opens the robot USD alone (same as Step 3b) and inspects the prim stack --
the list of every layer/reference arc that contributes to
/Root/turtlebot3_burger. If Geometry/Physics/Materials were meant to come
from a reference to another file, this will show that reference arc and
let us see its target path -- which tells us exactly what's missing.

Usage:
    /path/to/isaac-sim/python.sh step3c_inspect_robot_composition.py \
        --robot /path/to/turtlebot3_lidar.usd
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--robot", type=str, required=True,
                     help="Path to the saved robot asset USD.")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless}, experience="/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/apps/isaacsim.exp.full.kit") 

import omni.usd  # noqa: E402
from pxr import Usd  # noqa: E402

omni.usd.get_context().open_stage(args.robot)
stage = omni.usd.get_context().get_stage()

ROBOT_PRIM_PATH = "/Root/turtlebot3_burger"
prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)

print(f">>> Inspecting: {ROBOT_PRIM_PATH}")
print(f">>> IsValid: {prim.IsValid()}")

print(">>> --- Prim stack (every layer/spec contributing to this prim) ---")
for spec in prim.GetPrimStack():
    print(f"    layer: {spec.layer.identifier}")

print(">>> --- Composition query: reference/payload arcs on this prim ---")
query = Usd.PrimCompositionQuery(prim)
for arc in query.GetCompositionArcs():
    intro_layer = arc.GetIntroducingLayer()
    target_node = arc.GetTargetNode()
    print(f"    arc type: {arc.GetArcType()}")
    print(f"      introduced in layer: {intro_layer.identifier if intro_layer else None}")
    print(f"      target layer stack: {[l.identifier for l in target_node.layerStack.layers] if target_node else None}")

print(">>> --- Root layer's raw text (first 4000 chars) ---")
print(stage.GetRootLayer().ExportToString()[:4000])

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
