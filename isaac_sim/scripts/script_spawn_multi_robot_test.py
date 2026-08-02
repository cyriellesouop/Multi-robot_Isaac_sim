import argparse
import sys

# ------------------------------------------------------------------------------
# 1. CLI Arguments
# ------------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Spawn one or more robots in Isaac Sim 6.0.")
parser.add_argument("--env", type=str, required=True, help="Path to the saved environment USD.")
parser.add_argument("--robot", type=str, required=True, help="Path to the saved robot asset USD.")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots to spawn.")
parser.add_argument("--headless", action="store_true", help="Run in headless mode without GUI.")
parser.add_argument("--output", type=str, default=None, help="Optional path to save the composed stage.")
args = parser.parse_args()

# ------------------------------------------------------------------------------
# 2. Launch SimulationApp (Isaac Sim 6.0 with Full Kit Experience)
# ------------------------------------------------------------------------------
from isaacsim import SimulationApp

EXPERIENCE_PATH = "/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64/apps/isaacsim.exp.full.kit"

print(">>> Initializing Isaac Sim 6.0 SimulationApp...")
simulation_app = SimulationApp(
    {"headless": args.headless},
    experience=EXPERIENCE_PATH
)
print(">>> SimulationApp booted successfully.")

# Enable the Isaac Sim 6.0 ROS 2 Bridge Extension
import isaacsim.core.utils.extensions as extensions_utils
extensions_utils.enable_extension("isaacsim.ros2.bridge")

# ------------------------------------------------------------------------------
# 3. Imports (Must happen AFTER SimulationApp is instantiated)
# ------------------------------------------------------------------------------
import omni.usd
import numpy as np
from pxr import UsdGeom, Sdf
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage

# ------------------------------------------------------------------------------
# 4. Stage Setup & World Initialization
# ------------------------------------------------------------------------------
print(f">>> Opening environment stage: {args.env}")
success = omni.usd.get_context().open_stage(args.env)

if not success:
    print(f">>> ERROR: Failed to open environment stage at {args.env}")
    simulation_app.close()
    sys.exit(1)

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

# Grid Spacing Configuration
SPACING = 1.5  # meters between spawned robots
GRID_COLS = int(np.ceil(np.sqrt(args.num_robots)))

# ------------------------------------------------------------------------------
# 5. Robot Spawning Loop
# ------------------------------------------------------------------------------
for i in range(args.num_robots):
    namespace = f"robot_{i}"
    robot_prim_path = f"/{namespace}"

    # Calculate grid position (X, Y)
    x_pos = (i % GRID_COLS) * SPACING
    y_pos = (i // GRID_COLS) * SPACING
    z_pos = 0.05  # Hover 5cm above floor to prevent initial overlap physics jitter
    position = (x_pos, y_pos, z_pos)

    print(f">>> Spawning [{namespace}] at position {position}...")

    # A. Reference Robot USD to Stage
    add_reference_to_stage(usd_path=args.robot, prim_path=robot_prim_path)
    robot_prim = stage.GetPrimAtPath(robot_prim_path)

    if not robot_prim.IsValid():
        print(f">>> ERROR: Failed to spawn robot prim at {robot_prim_path}")
        continue

    # B. Clean up duplicate/stray differential_drive graph if present
    stray_path = f"{robot_prim_path}/differential_drive"
    stray_prim = stage.GetPrimAtPath(stray_path)
    if stray_prim.IsValid():
        print(f">>> Removing stray sibling graph at {stray_path}...")
        stage.RemovePrim(stray_path)

    # C. Set World Position Transformation
    xformable = UsdGeom.Xformable(robot_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(value=position)

    # D. Set ROS 2 Namespace via USD Attribute (Isaac Sim 6.0 standard)
    # Uses bare 'robot_i' without leading slash to avoid double-slashes in topics
    ns_attr = robot_prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String)
    ns_attr.Set(namespace)

    print(f">>> Successfully configured {namespace} with isaac:namespace = '{namespace}'")

# ------------------------------------------------------------------------------
# 6. Optional Stage Saving / Exporting
# ------------------------------------------------------------------------------
if args.output:
    print(f">>> Saving composed scene to {args.output}...")
    stage.GetRootLayer().Export(args.output)
    print(">>> Stage saved.")

# ------------------------------------------------------------------------------
# 7. World Reset (Triggers PhysX & Action Graph Compilation)
# ------------------------------------------------------------------------------
print(">>> Resetting World (Initializing Physics and OmniGraph ROS 2 nodes)...")
world.reset()
print(">>> Simulation initialized! Ready to process ROS 2 command topics.")

# ------------------------------------------------------------------------------
# 8. Main Physics & Rendering Loop
# ------------------------------------------------------------------------------
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()