"""
Open any robot/graph USD file directly (with full GUI) for inspection
and live editing. Automatically prints the top-level prim structure so
you can see the actual root path for THIS file, rather than assuming a
path that worked for a different file.

Usage:
    /path/to/isaac-sim/python.sh open_and_edit_usd.py \
        --robot /path/to/some_file.usd \
        --prim-path /Root/turtlebot3_burger   # optional, adjust per file
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--robot", type=str, required=True,
                     help="Path to the USD file to open.")
parser.add_argument("--prim-path", type=str, default=None,
                     help="Prim path to inspect in detail. If omitted, "
                          "only the top-level prim listing is shown.")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp(
    {"headless": args.headless},
    experience="/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/apps/isaacsim.exp.full.kit"
)

import omni.usd  # noqa: E402
from pxr import Usd  # noqa: E402

omni.usd.get_context().open_stage(args.robot)
stage = omni.usd.get_context().get_stage()

print(f">>> Default prim: {stage.GetDefaultPrim()}")
print(">>> Top-level prims in this file:")
for prim in stage.Traverse():
    if prim.GetPath().pathElementCount == 1:
        print(f"    {prim.GetPath()}  ({prim.GetTypeName()})")

if args.prim_path:
    prim = stage.GetPrimAtPath(args.prim_path)
    print(f">>> Inspecting: {args.prim_path}")
    print(f">>> IsValid: {prim.IsValid()}")

    if not prim.IsValid():
        print(">>> This path does not exist in this file -- check the")
        print(">>> top-level prim listing above for the correct root path.")
    else:
        print(">>> --- Prim stack (every layer/spec contributing to this prim) ---")
        for spec in prim.GetPrimStack():
            print(f"    layer: {spec.layer.identifier}")

        print(">>> --- Composition arcs on this prim ---")
        query = Usd.PrimCompositionQuery(prim)
        for arc in query.GetCompositionArcs():
            target_node = arc.GetTargetNode()
            print(f"    arc type: {arc.GetArcType()}")
            if target_node:
                print(f"      target layer stack: "
                      f"{[l.identifier for l in target_node.layerStack.layers]}")

print(">>> Keeping app open -- edit the graph now.")
print(">>> Close the window (or Ctrl+C here) when done.")

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
