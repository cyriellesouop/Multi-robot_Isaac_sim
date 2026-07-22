"""
Fix the broken relative payload paths inside the "Physics" variantSet's
mujoco/physics/physx options on turtlebot3_lidar.usd. Same broken-path
pattern as the Geometry reference and differential_drive payload fixed
earlier, but this time the payload is authored INSIDE a variant option,
so we need Usd's variant edit context to target the right spec rather
than editing the prim's default (outside-variant) content.

Usage:
    /path/to/isaac-sim/python.sh fix_physics_variant_payloads.py \
        --robot /path/to/turtlebot3_lidar.usd
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--robot", type=str, required=True,
                     help="Path to the robot USD file to fix, in place.")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Usd  # noqa: E402

ROBOT_PRIM_PATH = "/Root/turtlebot3_burger"
PAYLOADS_DIR = ("/home/UFAD/audreycyriell.mo/Documents/HRC/turtlebot/"
                "turtlebot3_description/urdf/tb3_burger_processed/payloads")

# Maps each variant name to its correct absolute payload path.
# "none" is skipped -- it has no payload to fix (empty variant).
VARIANT_FIXES = {
    "mujoco": f"{PAYLOADS_DIR}/Physics/mujoco.usda",
    "physics": f"{PAYLOADS_DIR}/Physics/physics.usda",
    "physx": f"{PAYLOADS_DIR}/Physics/physx.usda",
}

stage = Usd.Stage.Open(args.robot)
prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
print(f">>> Prim valid: {prim.IsValid()}")

vset = prim.GetVariantSets().GetVariantSet("Physics")
original_selection = vset.GetVariantSelection()
print(f">>> Original variant selection: '{original_selection}'")

for variant_name, correct_path in VARIANT_FIXES.items():
    print(f">>> Fixing variant '{variant_name}'...")
    vset.SetVariantSelection(variant_name)

    with vset.GetVariantEditContext():
        # Editing within this context targets the selected variant's own
        # spec, not the prim's outside-variant content.
        payloads = prim.GetPayloads()
        current = prim.GetMetadata("payload")
        print(f"    current payload metadata: {current}")

        payloads.ClearPayloads()
        payloads.AddPayload(assetPath=correct_path)
        print(f"    set payload to: {correct_path}")

# Restore the original selection before saving, so nothing else changes.
vset.SetVariantSelection(original_selection)
print(f">>> Restored variant selection to: '{original_selection}'")

stage.GetRootLayer().Save()
print(">>> Saved. All three variant payloads updated via USD API (binary-safe).")

simulation_app.close()
