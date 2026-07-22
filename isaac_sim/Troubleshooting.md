# Single-Robot Spawn: Problems Encountered and Fixes

This documents every issue hit while getting one TurtleBot3 (with lidar +
`differential_drive`) to spawn and be recognized correctly by Isaac Sim
6.0.0, before attempting to scale to multiple robots. Kept as a reference
so the same issues (or their fixes) are recognizable if they recur when
scaling to N robots.

## 1. `omni.isaac.core` module not found

**Symptom:**
```
ModuleNotFoundError: No module named 'omni.isaac.core'
```

**Cause:** Isaac Sim renamed its entire `omni.isaac.*` extension namespace
to `isaacsim.*` starting in version 4.5. Any code/tutorial written before
that rename uses the old names.

**Fix:** Replace `omni.isaac.X` imports with `isaacsim.X`, e.g.:
```python
# Old:
from omni.isaac.core.utils.stage import add_reference_to_stage
# New:
from isaacsim.core.utils.stage import add_reference_to_stage
```

---

## 2. Robot spawns with missing Geometry/Physics/Materials

**Symptom:** Referencing `turtlebot3_lidar.usd` (or opening it standalone)
showed only `Graphs/differential_drive` under `turtlebot3_burger` --
`Geometry`, `Physics`, `Materials` were completely absent, with no
error dialog, just missing content. Stage panel showed "Missing
references found" on `turtlebot3_burger`.

**Cause:** The robot's actual chassis/wheel/sensor geometry comes from a
**reference** to an external file (`base.usda`), authored with a
**relative** path (`../turtlebot/turtlebot3_description/...`). That path
was relative to wherever the robot was originally built, and broke once
`turtlebot3_lidar.usd` was saved into `isaac_sim/assets/` -- a different
directory. `over` blocks (used throughout this file) only take effect if
the thing they override actually exists; with the reference broken, they
had nothing to attach to, so entire subtrees vanished silently instead of
showing as visibly "broken."

**Diagnosis:** Used `Usd.PrimCompositionQuery` on the affected prim to
inspect its composition arcs, and `stage.GetRootLayer().ExportToString()`
to read the raw file content and see the literal broken reference path.

**Fix:** Located the real file with `find`, then replaced the broken
relative reference with the correct absolute path -- via the USD API
(`Sdf.Reference`), not a text editor or `sed` (see issue #5 below for why).

---

## 3. `differential_drive` graph shows no child nodes

**Symptom:** Same "missing content" pattern as #2, but on
`Graphs/differential_drive` instead -- no `on_playback_tick`,
`differential_controller`, etc. visible.

**Cause:** Same root cause as #2, different composition mechanism: this
content comes from a **payload** (`prepend payload =
@./differential_drive.usd@`), also using a broken relative path.

**Fix:** Same approach as #2, but editing `payloadList` via `Sdf.Payload`
instead of `referenceList`/`Sdf.Reference` (payloads and references are
distinct composition arc types with separate APIs).

---

## 4. `Physics` prim shows zero children (articulation fails)

**Symptom:** Even after fixing #2 and #3, `turtlebot3_burger` still
showed "Missing references found," and `Physics` remained an empty
Scope. Once ROS2/physics was live, this caused a real (not just
cosmetic) failure:
```
Failed to find articulation at '/robot_0/turtlebot3_burger'
Pattern did not match any articulations
Articulation controller failed... object of type 'NoneType' has no len()
```

**Cause:** The robot's `Physics` variant set (`mujoco` / `none` /
`physics` / `physx`) has its own broken relative payload **inside each
variant option**, pointing at `payloads/Physics/<name>.usda`. This is a
third, independent instance of the same relative-path bug, nested one
level deeper (inside a variant, not the prim's default content) --
easy to miss since standard composition-arc queries on the prim don't
surface it without first selecting into the variant.

**Diagnosis:** Searching the full raw text of `turtlebot3_lidar.usd` for
the literal `variantSet "Physics" = {` block (not just the `variantSets =
"Physics"` *selection* line, which is a different, easily-confused
string) revealed all three variants' broken payloads at once.

**Fix:** Used `vset.GetVariantEditContext()` to enter each variant
(`mujoco`, `physics`, `physx`) in turn, cleared and re-added the payload
with the correct absolute path via the USD API, then restored the
original variant selection before saving.


---

## 5. `colcon build` fails: `cmake` missing shared library

**Symptom:**
```
/tools/Xilinx/Vitis/2023.1/tps/lnx64/cmake-3.3.2/bin/cmake: error while
loading shared libraries: libidn.so.11: cannot open shared object file
```

**Cause:** A Xilinx Vitis environment setup script (sourced via
`.bashrc`) prepends its own bundled toolchain directories -- including an
old, broken `cmake` -- ahead of `/usr/bin` in `PATH`.

**Fix:** Force the system `cmake` first for the build step only:
```bash
export PATH="/usr/bin:$PATH"
```
placed in `run.sh` right before `colcon build`, without permanently
altering the shell (so Vitis's own tools stay available for FPGA work).

---

## 6. `rclpy` fails to import (`No module named 'rclpy._rclpy_pybind11'`)

**Symptom:**
```
Could not import system rclpy: No module named 'rclpy._rclpy_pybind11'
The C extension '.../python3.10/site-packages/_rclpy_pybind11.cpython-312-...so' isn't present
```
Same error for both "system" and "internal" rclpy load attempts.

**Cause:** ROS2 Humble's system-installed `rclpy` is compiled for Python
3.10, but Isaac Sim 6.0.0 bundles its own Python 3.12 -- incompatible
ABI. Isaac Sim does bundle a matching Python-3.12 `rclpy` build
internally (found via `find ... -iname "*rclpy*pybind11*cpython-312*"`),
but by default the *system* `/opt/ros/humble/...` path is found first on
Python's module search path, so the broken one always loads first
regardless.

**Fix:** Two separate variables for two separate resolution mechanisms --
`PYTHONPATH` (Python's module search) must be **prepended** (not
appended) with the bundled path so it's found before the system one;
`LD_LIBRARY_PATH` (shared C library search) needs the bundled `lib/`
folder for rclpy's underlying C++ dependencies:
```bash
export PYTHONPATH="<isaac_sim>/exts/isaacsim.ros2.core/humble/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:<isaac_sim>/exts/isaacsim.ros2.core/humble/lib"
```

---

## 7. Active conda environment interferes with Isaac Sim

**Symptom:** `python.sh` itself warns:
```
Warning: running in conda env, please deactivate before executing this script
```
and Isaac Sim crashed with native errors that had nothing obviously to do
with the actual Python code.

**Cause:** Conda environments inject their own versions of common shared
libraries, which can silently conflict with the specific versions Isaac
Sim's bundled Kit runtime expects.

**Fix:** `conda deactivate` before launching Isaac Sim (standalone or
GUI). Confirmed by prompt no longer showing `(base)`/env name prefix.




---

## 8. Viewport appears completely black / empty

**Symptom:** Script ran without errors, robot and environment referenced
successfully, but the Isaac Sim viewport showed nothing.

**Cause:** The camera simply wasn't pointed at the spawned content --
scripted stage-opening/referencing doesn't automatically frame the
viewport on new content.

**Fix:** Select the robot (or any relevant prim) in the Stage panel, then
press **F** (Frame Selected) with the viewport focused.

---

## Current status

- Geometry, `differential_drive` graph, and Physics/articulation are all
  now structurally correct and confirmed via `Full prim hierarchy`
  dumps and the absence of "Missing references found" in the Stage panel.
- PhysX correctly recognizes the robot as an articulation (confirmed: no
  more `Failed to find articulation` / `Pattern did not match any
  articulations` errors after hitting Play).

