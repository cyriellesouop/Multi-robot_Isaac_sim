"""
Step 1: Does Isaac Sim boot at all?

No stage loading, no robot spawning -- just start SimulationApp and
immediately close it. If this hangs or never reaches "Boot test passed",
the problem is in Isaac Sim's startup itself (GPU driver, shader cache,
installation), not in our spawner logic.

Usage:
    /path/to/isaac-sim/python.sh step1_boot_test.py
    /path/to/isaac-sim/python.sh step1_boot_test.py --headless
"""

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--env", type=str, required=True, help="Path to the saved environment USD.")
parser.add_argument("--headless", action="store_true") # headless controls whether a GUI window opens when lauching the APP
parser.add_argument("--robot", type=str, required=True,help="Path to the saved robot asset USD.")
args = parser.parse_args()

# SimulationApp is what actually boots the whole Kit application; nothing else in the omni.*/isaacsim.*/pxr namespace exists until this runs.
from isaacsim import SimulationApp  

print(">>> Creating SimulationApp...")
simulation_app = SimulationApp({"headless": args.headless}) # The boot call that produce : Simulation App Starting output
print(">>> SimulationApp created successfully.")

#from isaacsim.core.utils.extensions import enable_extension
#enable_extension("isaacsim.ros2.bridge")  # enable the ROS2 Bridge extension. It turn on the actual code that implements what a "ROS2Context" node 

'''
#it's Omniverse's USD-stage-management API (opening/getting the current stage, etc.)

UsdGeom — USD's geometry module. We use it for UsdGeom.Xformable, which lets us read/write an object's transform (position, rotation, scale).

Sdf — USD's "Scene Description Foundation" module. We use it for Sdf.ValueTypeNames.String, 
which tells USD "the attribute I'm about to create holds a string value" (as opposed to a number, bool, etc.) 
USD attributes are strongly typed, so you have to declare the type when creating one.

add_reference_to_stage — the Isaac Sim convenience function that does the actual "import this other USD file into my current stage at this location" operation. 
Equivalent to  drag-and-drop referencing in the GUI.
'''
import omni.usd 
from pxr import UsdGeom, Sdf #
from isaacsim.core.utils.stage import add_reference_to_stage


'''
get_context() returns the current USD "context" (the object managing which stage is loaded). 
open_stage(path) loads your SimpleRoom_office.usd file into it — this is the GUI's "File > Open" equivalent, done in code. 
Returns True/False depending on whether loading succeeded.
'''
print(f">>> Opening stage: {args.env}")
success = omni.usd.get_context().open_stage(args.env)
print(f">>> open_stage returned: {success}")

'''
Fetches the actual Usd.Stage object now that it's loaded.
this is the in-memory representation of the scene graph (all your prims: SimpleRoom, PhysicsScene
'''
stage = omni.usd.get_context().get_stage()
print(f">>> Stage object: {stage}")

'''
Three plain variables, not USD objects yet — just deciding where in the stage the robot will live
(/robot_0 — a new top-level prim), what to name its ROS2 namespace, and where to place it in 3D space
'''
# --- Spawn exactly one robot -------------------------------------------
ROBOT_PRIM_PATH = "/robot_0"
NAMESPACE = "robot_0"
POSITION = (0.0, 0.0, 0.0)

'''
This is the actual spawn. It reads turtlebot3_lidar.usd from disk and inserts its entire prim hierarchy  into the currently-open stage, rooted at /robot_0. 
Before this line, /robot_0 doesn't exist in the stage at all; after it, the whole robot exists there.
'''
print(f">>> Referencing robot USD at {ROBOT_PRIM_PATH}")
add_reference_to_stage(usd_path=args.robot, prim_path=ROBOT_PRIM_PATH)

'''
add_reference_to_stage doesn't hand you back a usable Python object for the new prim — it just modifies the stage. 
So this line separately fetches that prim by its path so we can then do things to it (set its position, add an attribute).
'''
robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
print(f">>> robot_prim.IsValid(): {robot_prim.IsValid()}") #To catch that failure mode instead of it silently doing nothing 

if not robot_prim.IsValid():
    print(">>> ERROR: robot prim is not valid, stopping here.")
else:
    xformable = UsdGeom.Xformable(robot_prim) #"Wraps" the raw prim in USD's Xformable interface. This is the object that actually knows how to read/write transforms (position, rotation, scale).
    xformable.ClearXformOpOrder() #Removes any existing transform operations the referenced robot might already carry from its own saved file 
    xformable.AddTranslateOp().Set(value=POSITION) #Adds a fresh "translate" transform operation and sets it to POSITION(move to XYZ coordinate)
    print(f">>> Set position to {POSITION}")


    '''
    Two steps: first, declare a new custom attribute on the robot's root prim, named isaac:namespace, typed as a string (that's what Sdf.ValueTypeNames.String specifies). 
    Then, set its value to "robot_0". Every ROS2 OmniGraph node nested inside this prim's subtree reads this attribute at runtime to know what to prefix its topic names with.
    '''
    #ns_attr = robot_prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.String)
    #ns_attr.Set(NAMESPACE)



#if stage is not None:
 #   print(">>> Top-level prims in the stage:")
  #  for prim in stage.Traverse(): #stage.Traverse() walks every prim in the scene, at every nesting depth.
   #     if prim.GetPath().pathElementCount == 1: ## Only print top-level (direct children of /) prims to keep this readable ather than dumping every nested child.
    #        print(f"    {prim.GetPath()}  ({prim.GetTypeName()})")

print(">>> Keeping app open so you can look at the Isaac Sim window.")

'''
This is the actual "keep the window open" loop. is_running() is True as long as the app hasn't been told to close. 
update() advances one simulation/render frame. without calling this repeatedly, 
the window would appear frozen or never render at all, since nothing would be pumping the render loop.
'''
while simulation_app.is_running():
    simulation_app.update()


simulation_app.close() #Once the loop exits (you closed the window, or Ctrl+C interrupted it), this shuts Kit down cleanly.
