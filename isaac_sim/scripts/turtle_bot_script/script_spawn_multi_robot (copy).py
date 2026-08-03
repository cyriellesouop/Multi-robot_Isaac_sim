"""
Multi-robot fleet spawner for Isaac Sim.

Loads a saved environment (SimpleRoom_office.usd) and references a saved,
self-contained robot asset (turtlebot3_lidar.usd) N times at auto-generated
grid positions, applying an isaac:namespace attribute to each robot's root
prim so every ROS2 OmniGraph node nested inside it (differential_drive,
lidar publisher, etc.) automatically publishes/subscribes under a unique
namespace -- no manual per-node editing required.

This is a standalone script (boots Isaac Sim itself via SimulationApp),
so fleet size and paths are passed as command-line arguments instead of
being hardcoded -- convenient for running the same script across your
1/2/3/4-robot trials without editing the file each time.

Usage:
    python spawn_multi_robot_fleet.py --num-robots 4 \
        --env /path/to/SimpleRoom_office.usd \
        --robot /path/to/turtlebot3_lidar.usd

    # Headless (no GUI window), useful for batch/CI runs:
    python spawn_multi_robot_fleet.py --num-robots 4 --headless

    # Keep the sim running after spawning instead of closing immediately:
    python spawn_multi_robot_fleet.py --num-robots 4 --keep-open
"""

import argparse

from isaacsim import SimulationApp


def parse_args():
    parser = argparse.ArgumentParser(description="Spawn a robot fleet in Isaac Sim.")
    parser.add_argument("--num-robots", type=int, required=True,
                         help="Number of robots to spawn in the fleet.")
    parser.add_argument("--env", type=str, default="/path/to/SimpleRoom_office.usd",
                         help="Path to the saved environment USD.")
    parser.add_argument("--robot", type=str, default="/path/to/turtlebot3_lidar.usd",
                         help="Path to the saved robot asset USD.")
    parser.add_argument("--spacing", type=float, default=1.5,
                         help="Distance in meters between grid spawn points.")
    parser.add_argument("--headless", action="store_true",
                         help="Run without opening a GUI window.")
    parser.add_argument("--keep-open", action="store_true",
                         help="Keep Isaac Sim open after spawning instead of closing.")
    return parser.parse_args()


args = parse_args()

# SimulationApp must be created before any omni.* / pxr imports.
simulation_app = SimulationApp({"headless": args.headless})

from pxr import Usd, UsdGeom, Sdf  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

ROBOT_PRIM_PATH_TEMPLATE = "/World/robot_{i}"
NAMESPACE_TEMPLATE = "robot_{i}"


def load_environment(env_path: str) -> Usd.Stage:
    """Open the saved environment stage."""
    omni.usd.get_context().open_stage(env_path)
    return omni.usd.get_context().get_stage()


def grid_positions(n: int, spacing: float):
    """Generate n (x, y, z) spawn points on a roughly square grid."""
    cols = int(n ** 0.5) + (1 if int(n ** 0.5) ** 2 < n else 0)
    return [(spacing * (i % cols), spacing * (i // cols), 0.0) for i in range(n)]


def spawn_robot(stage: Usd.Stage, robot_usd_path: str, prim_path: str,
                 position: tuple, namespace: str):
    """Reference the robot asset into the stage and namespace it."""
    add_reference_to_stage(usd_path=robot_usd_path, prim_path=prim_path)

    robot_prim = stage.GetPrimAtPath(prim_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Failed to reference robot at {prim_path}")

    xformable = UsdGeom.Xformable(robot_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(value=position)

    # Applying isaac:namespace on the ROOT prim propagates to every ROS2
    # OmniGraph node nested inside it -- no per-node editing needed.
    ns_attr = robot_prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String)
    ns_attr.Set(namespace)

    print(f"Spawned {prim_path} at {position} with namespace '{namespace}'")


def main():
    stage = load_environment(args.env)
    positions = grid_positions(args.num_robots, args.spacing)

    for i in range(args.num_robots):
        prim_path = ROBOT_PRIM_PATH_TEMPLATE.format(i=i)
        namespace = NAMESPACE_TEMPLATE.format(i=i)
        spawn_robot(stage, args.robot, prim_path, positions[i], namespace)

    print(f"Done. Spawned {args.num_robots} robots.")
    print("Run `ros2 topic list` in a ROS2-sourced terminal to verify each")
    print("robot's topics appear under its own namespace, e.g. /robot_0/cmd_vel.")

    if args.keep_open:
        while simulation_app.is_running():
            simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
