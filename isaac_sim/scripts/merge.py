"""
Merge ROS2 Context + ROS2 Subscribe Twist + On Playback Tick directly
into differential_drive.usd, eliminating the cross-file connection
entirely. Simpler, more robust for now -- clean file separation can be
revisited later once there's less debugging fatigue.

Usage:
    /path/to/isaac-sim/python.sh merge_ros2_into_differential_drive.py --headless
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
import omni.graph.core as og  # noqa: E402

DIFF_DRIVE_FILE = ("/home/UFAD/audreycyriell.mo/Documents/HRC/isaac-sim-tutorial/"
                    "multi_robot_lidar/isaac_sim/assets/differential_drive.usd")
GRAPH_PATH = "/Root/differential_drive"

omni.usd.get_context().open_stage(DIFF_DRIVE_FILE)
stage = omni.usd.get_context().get_stage()

print(">>> Adding on_playback_tick, ros2_context, ros2_subscribe_twist nodes...")
og.Controller.edit(
    GRAPH_PATH,
    {
        og.Controller.Keys.CREATE_NODES: [
            ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
            ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
            ("ros2_subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ],
        og.Controller.Keys.CONNECT: [
            (f"{GRAPH_PATH}/on_playback_tick.outputs:tick",
             f"{GRAPH_PATH}/ros2_subscribe_twist.inputs:execIn"),
            (f"{GRAPH_PATH}/ros2_context.outputs:context",
             f"{GRAPH_PATH}/ros2_subscribe_twist.inputs:context"),
            (f"{GRAPH_PATH}/ros2_subscribe_twist.outputs:angularVelocity",
             f"{GRAPH_PATH}/break_3_vector.inputs:tuple"),
            (f"{GRAPH_PATH}/ros2_subscribe_twist.outputs:linearVelocity",
             f"{GRAPH_PATH}/scale_to_from_stage_units.inputs:value"),
        ],
    },
)

for _ in range(10):
    simulation_app.update()

stage.GetRootLayer().Save()
print(">>> Saved.")

# Verify
graph_prim = stage.GetPrimAtPath(GRAPH_PATH)
print(">>> Children now:")
for child in graph_prim.GetChildren():
    print(f"    {child.GetName()}")

break_attr = stage.GetPrimAtPath(f"{GRAPH_PATH}/break_3_vector").GetAttribute("inputs:tuple")
print(f">>> break_3_vector.inputs:tuple connections: {[str(c) for c in break_attr.GetConnections()]}")

simulation_app.close()
