## Execution Guide: Running a Trial and Capturing Metrics

This section documents, command by command, the full sequence to run one
trial (fixed robot count R, sensor tier S, repetition k) and produce a
row in `results_summary.csv`. Repeat the "Per-trial execution" section
once per `(R, S, rep)` combination.

> **Status note (as of this writing):** this guide describes the
> intended, validated workflow. One trial (`R1_S-low_rep1`) has been
> run and successfully processed end-to-end, but it was collected
> **before** the goal configuration was switched to fixed goals (see
> "Goal design" below) — it used `RandomGoalGenerator`, not the fixed
> 3-goal set. No trial has yet been collected under the finalized
> fixed-goal configuration. Baseline checkpoint files and per-trial
> resource CSVs following the naming convention below have not yet
> been created on disk — see the per-item status markers throughout.

### One-time environment setup

Already handled automatically via `~/.bashrc` for every new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/HRC/IsaacSim-ros_workspaces/humble_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42
```
Verify any new terminal picked these up:
```bash
echo $ROS_DISTRO          # expect: humble
echo $ROS_DOMAIN_ID       # expect: 42
ros2 pkg list | grep carter_navigation   # expect: carter_navigation
```

**Not** in `.bashrc` (set manually, per terminal, only when actually
running the DDS-vendor-swap stretch experiment):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp     # default if unset
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # Cyclone DDS alternative
```

**Conda/PATH shadowing — two distinct issues, two distinct fixes.**
Both stem from conda's `(base)` environment taking priority over system
tools, but they surface in different places and are documented
separately in `isaac_sim/Troubleshooting.md` (issues #5 and #7; the
`analyze_trial.py` case below is a related but separately-documented
third instance — see status note):

- **Before `colcon build`** (Troubleshooting.md #5 — Xilinx Vitis'
  bundled `cmake` shadows the system one):
  ```bash
  export PATH="/usr/bin:$PATH"
  ```
- **Before launching Isaac Sim itself** (Troubleshooting.md #7 — conda
  libraries conflict with Isaac Sim's bundled Kit runtime):
  ```bash
  conda deactivate
  ```
- **Before running `analyze_trial.py`** (conda's Python lacks ROS 2's
  compiled `rosbag2_py` bindings — same root cause as #7, different
  symptom, not yet written up as its own numbered entry in
  `Troubleshooting.md`):
  ```bash
  conda deactivate
  # or: /usr/bin/python3 analyze_trial.py ...
  ```

### Per-condition scene setup (once per sensor tier, reused across reps)

1. Launch Isaac Sim:
   ```bash
   cd ~/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64
   ./isaac-sim.sh
   ```
2. Load the target scene (`Nav2_Nova_Carter_robot_warehouse.usd` for
   R=1; multi-robot scene TBD for R=2/R=3).
3. Set the sensor tier by activating/deactivating the relevant OmniGraph
   publisher nodes (see "Sensor-load tiers" section above) — **not**
   the physical sensor prims. **Status: no dedicated saved USD variants
   per tier exist yet** — tiers are currently set by manually toggling
   nodes in the running scene, not loaded from separate files. If/when
   saved as separate scenes, record their paths here.
4. Confirm topic set matches the intended tier:
   ```bash
   ros2 topic list
   ```

### Baseline checkpoint capture (per condition, before the first rep)

**Status: not yet performed for any condition.** The folder
`isaac_sim/scripts/checkpoints_CPU_GPU_measurements/` exists as the
intended destination but is currently empty. The commands below were
run once, ad hoc, during pipeline validation (see `idle_baseline.md`
for that one-off idle-system capture) but not yet repeated per
condition using the convention below.

**Checkpoint A — scene loaded, not playing:**
```bash
{ echo "=== free -m ==="; free -m; echo; \
  echo "=== nvidia-smi ==="; nvidia-smi; echo; \
  echo "=== top ==="; top -bn1 | head -20; } \
  > checkpoints_CPU_GPU_measurements/baseline_R{n}_S-{tier}_loaded-paused.txt
```

**Checkpoint B — press Play, simulation running, no goals sent yet:**
Wait ~10-15s for stabilization, then repeat with `_playing-idle.txt`.

(Redirecting to a file, not just reading the terminal, is the part that
was missed previously — running the commands alone does not save
anything.)

### Per-trial execution (repeat for rep1, rep2, rep3, ...)

Run the following from a single working directory
(`isaac_sim/scripts/nova_carter_data/`), each command in its own
terminal, all sourced per the environment setup above.

**Terminal 1 — resource sampler** (start first, keep running through
the whole trial):
```bash
./resource_sampler.sh trial_R{n}_S-{tier}_rep{k}_resources.csv 1
```
**Status/naming caution:** the existing `R1_S-low_rep1` trial has its
resource data saved as a generically-named `resources.csv` inside the
bag folder itself (left over from initial script validation, alongside
an even earlier `test_run.csv`). Neither follows this convention. If
reusing that data, rename and move it out of the bag folder first:
```bash
mv trial_R1_S-low_rep1_bag/resources.csv trial_R1_S-low_rep1_resources.csv
```

**Terminal 2 — rosbag record** (topics depend on active sensor tier —
add `/front_3d_lidar/lidar_points` and `/chassis_imu/imu` for Medium,
plus camera topics for High):
```bash
ros2 bag record -o trial_R{n}_S-{tier}_rep{k}_bag /tf /scan /clock /chassis/odom
```

**Terminal 3 — Nav2 (authoritative success/failure source):**
```bash
ros2 launch carter_navigation carter_navigation.launch.py 2>&1 | tee trial_R{n}_S-{tier}_rep{k}_nav2.log
```

**Terminal 4 — navigation goal sequence** (start last, once Nav2 is
fully up — wait for RViz2 / costmaps to finish initializing):
```bash
ros2 launch isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.py 2>&1 | tee trial_R{n}_S-{tier}_rep{k}_navgoal.log
```
Requires `goal_generator_type: "GoalReader"` and a `goals.txt`
containing the three fixed goals (short-range, long-traverse,
obstacle-avoidance — see "Goal design" above). **Status: this launch
file / goals.txt edit has been designed and agreed on but not yet
confirmed written to disk** — verify `goals.txt` actually contains the
three finalized coordinate lines before relying on this step producing
comparable data across conditions.

**Wait for Terminal 4 to exit on its own** (`"process has finished
cleanly"` — do not Ctrl+C this one; let all 3 configured goals complete
or fail naturally).

**Then stop, in this order:**
1. Ctrl+C Terminal 2 (rosbag record)
2. Ctrl+C Terminal 1 (resource sampler)
3. Ctrl+C Terminal 3 (Nav2) — safe to stop now, trial is over

### Post-processing — one command per trial

```bash
python3 analyze_trial.py \
  --bag trial_R{n}_S-{tier}_rep{k}_bag \
  --navlog trial_R{n}_S-{tier}_rep{k}_navgoal.log \
  --nav2log trial_R{n}_S-{tier}_rep{k}_nav2.log \
  --robots {n} --tier {tier} --rep {k} \
  --summary results_summary.csv
```
(Run with `conda deactivate` active first, or via `/usr/bin/python3` —
see environment setup above.)

This appends one row to `results_summary.csv` (creating it with a
header on the first call). Repeat for every `(R, S, rep)` combination —
`results_summary.csv` accumulates every trial's row, and is the single
file your final analysis/plotting scripts read from.

### Output files: what each contains, and what consumes it

| File | Status | Produced by | Contains | Consumed by |
|---|---|---|---|---|
| `checkpoints_CPU_GPU_measurements/baseline_R{n}_S-{tier}_loaded-paused.txt` | **not yet created** | manual, redirected (`free -m`/`nvidia-smi`/`top`) | Idle-with-scene-loaded CPU/RAM/GPU snapshot | Manual reference; optional row in Table 2 |
| `checkpoints_CPU_GPU_measurements/baseline_R{n}_S-{tier}_playing-idle.txt` | **not yet created** | manual, same commands | Playing-but-no-goals CPU/RAM/GPU snapshot | Same as above |
| `idle_baseline.md` | exists (one-off) | manual, one-time (post-restart) | Fully idle system reference (no Isaac Sim at all) | Reference point for "Idle" row |
| `trial_R{n}_S-{tier}_rep{k}_resources.csv` | **only exists for rep1, misnamed/misplaced** | `resource_sampler.sh` | 1 Hz CPU%, RAM MiB, GPU util%, GPU mem MiB, timestamped | **Not yet auto-consumed by `analyze_trial.py`** — integration pending |
| `trial_R{n}_S-{tier}_rep{k}_bag/` | exists for rep1 (random-goal config) | `ros2 bag record` | Raw `/tf`, `/scan`, `/clock`, `/chassis/odom` (+ tier-dependent topics) | `analyze_trial.py` → RTF, sim/wall elapsed time, path length, message counts |
| `trial_R{n}_S-{tier}_rep{k}_navgoal.log` | exists for rep1 | `isaac_ros_navigation_goal` launch, via `tee` | Goal-generated / accepted / "Result" timestamps | `analyze_trial.py` → goal count, per-goal duration; **not reliable for success/failure** |
| `trial_R{n}_S-{tier}_rep{k}_nav2.log` | exists for rep1 | `carter_navigation` (Nav2) launch, via `tee` | `[bt_navigator]: Goal succeeded` / `Goal failed` (authoritative) | `analyze_trial.py` → true `n_succeeded`, `n_failed`, `success_rate` |
| `results_summary.csv` | exists, 1 row so far | `analyze_trial.py` (appended per trial) | One row per trial, tagged by `(robots, sensor_tier, rep)` | Final analysis / plotting — **master results file for the paper** |

> **Known gap:** `resources.csv` (CPU/RAM/GPU) is captured per trial
> but not yet folded into `results_summary.csv` by `analyze_trial.py`.
> Planned extension: a `--resources` argument computing mean/max
> CPU/RAM/GPU over the trial window, appended to the same summary row.

### Naming convention reference

```
checkpoints_CPU_GPU_measurements/baseline_R{n}_S-{tier}_loaded-paused.txt
checkpoints_CPU_GPU_measurements/baseline_R{n}_S-{tier}_playing-idle.txt
trial_R{n}_S-{tier}_rep{k}_bag/
trial_R{n}_S-{tier}_rep{k}_resources.csv
trial_R{n}_S-{tier}_rep{k}_navgoal.log
trial_R{n}_S-{tier}_rep{k}_nav2.log
```
where `{n}` = robot count (1, 2, 3), `{tier}` = sensor tier (low,
medium, high), `{k}` = repetition number (1, 2, 3, ...).
