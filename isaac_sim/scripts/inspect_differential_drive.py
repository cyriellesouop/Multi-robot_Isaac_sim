"""
Fix differential_drive.usd properly:
1. Remove the stray top-level /differential_drive prim (created by an
   earlier wrong-path edit) along with its garbage Graphs_01/Graphs_02
   instance-path overrides.
2. Re-copy the correct, fully-updated graph from turtlebot3_lidar.usd
   into the CORRECT destination path this time: /Root/differential_drive
   (nested under the Root Xform, matching this file's actual structure),
   not /differential_drive at the top level.

Usage:
    /path/to/isaac-sim/python.sh fix_differential_drive_file.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

from pxr import Sdf  # noqa: E402

SOURCE_FILE = "/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/multi_robot_lidar/isaac_sim/assets/turtlebot3_lidar.usd"
SOURCE_PATH = Sdf.Path("/Root/turtlebot3_burger/Graphs/differential_drive/differential_drive")

DEST_FILE = "/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/differential_drive.usd"
DEST_PATH_CORRECT = Sdf.Path("/Root/differential_drive")   # the REAL, correct path
STRAY_PATH = Sdf.Path("/differential_drive")                 # the wrong, stray duplicate

dest_layer = Sdf.Layer.FindOrOpen(DEST_FILE)
if dest_layer is None:
    raise RuntimeError(f"Could not open {DEST_FILE}")

# --- Step 1: remove the stray top-level prim entirely ---
stray_spec = dest_layer.GetPrimAtPath(STRAY_PATH)
if stray_spec is not None:
    print(f">>> Removing stray prim at {STRAY_PATH}")
    parent_spec = dest_layer.GetPrimAtPath(STRAY_PATH.GetParentPath())
    if parent_spec is not None:
        del parent_spec.nameChildren[stray_spec.name]
    else:
        del dest_layer.rootPrims[stray_spec.name]
    print(">>> Stray prim removed.")
else:
    print(f">>> No stray prim found at {STRAY_PATH} (already clean).")

# --- Step 2: re-copy the correct, fully-updated graph to the CORRECT path ---
source_layer = Sdf.Layer.FindOrOpen(SOURCE_FILE)
if source_layer is None:
    raise RuntimeError(f"Could not open {SOURCE_FILE}")

source_spec = source_layer.GetPrimAtPath(SOURCE_PATH)
if source_spec is None:
    raise RuntimeError(f"No prim spec found at {SOURCE_PATH} in {SOURCE_FILE}")

print(f">>> Copying {SOURCE_PATH} (from {SOURCE_FILE})")
print(f">>>   to    {DEST_PATH_CORRECT} (in {DEST_FILE}) -- the CORRECT nested path")

success = Sdf.CopySpec(source_layer, SOURCE_PATH, dest_layer, DEST_PATH_CORRECT)
print(f">>> Sdf.CopySpec succeeded: {success}")

if success:
    dest_layer.Save()
    print(f">>> Saved {DEST_FILE}")
else:
    print(">>> Copy failed -- nothing was saved.")

simulation_app.close()
