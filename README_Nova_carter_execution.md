## Execution Guide: Running a Trial and Capturing Metrics

This section documents, command by command, the full sequence to run one
trial (fixed robot count R, sensor tier S, repetition k) and produce a
row in `results_summary.csv`. Repeat the "Per-trial execution" section
once per `(R, S, rep)` combination.

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

**PATH shadowing fix** — required before `colcon build`, and before
running `analyze_trial.py` (conda's Python lacks the compiled
`rosbag2_py` bindings ROS 2 needs):
```bash
conda deactivate
# or, without leaving conda active:
export PATH=/usr/bin:$PATH
```

### Per-condition scene setup (once per sensor tier, reused across reps)

1. Launch Isaac Sim:
   ```bash
   cd ~/Documents/HRC/isaac-sim-standalone-6.0.0-linux-x86_64
   ./isaac-sim.sh
   ```
2. Load the target scene (`Nav2_Nova_Carter_robot_warehouse.usd` for
   R=1, or the appropriate multi-robot scene for R=2/R=3).
3. Set the sensor tier by activating/deactivating the relevant OmniGraph
   publisher nodes (see "Sensor-load tiers" section above) — **not**
   the physical sensor prims.
4. Confirm topic set matches the intended tier:
   ```bash
   ros2 topic list
   ```

### Baseline checkpoint capture (per condition, before the first rep)

**Checkpoint A — scene loaded, not playing:**
```bash
free -m
nvidia-smi
top -bn1 | head -20
```
Save this output as `baseline_R{n}_S-{tier}_loaded-paused.txt`.

**Checkpoint B — press Play, simulation running, no goals sent yet:**
Wait ~10-15s for stabilization, then repeat the same three commands.
Save as `baseline_R{n}_S-{tier}_playing-idle.txt`.

### Per-trial execution (repeat for rep1, rep2, rep3, ...)

Run the following from a single working directory
(`isaac_sim/scripts/nova_carter_data/`), each command in its own
terminal, all sourced per the environment setup above.

**Terminal 1 — resource sampler** (start first, keep running through
the whole trial):
```bash
./resource_sampler.sh trial_R{n}_S-{tier}_rep{k}_resources.csv 1
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

This appends one row to `results_summary.csv` (creating it with a
header on the first call). Repeat for every `(R, S, rep)` combination —
`results_summary.csv` accumulates every trial's row, and is the single
file your final analysis/plotting scripts read from.

### Output files: what each contains, and what consumes it

| File | Produced by | Contains | Consumed by |
|---|---|---|---|
| `baseline_R{n}_S-{tier}_loaded-paused.txt` | manual (`free -m`/`nvidia-smi`/`top`) | Idle-with-scene-loaded CPU/RAM/GPU snapshot | Manual reference; optional row in Table 2 (experimental configuration) |
| `baseline_R{n}_S-{tier}_playing-idle.txt` | manual, same commands | Playing-but-no-goals CPU/RAM/GPU snapshot | Same as above |
| `idle_baseline.md` | manual, one-time (post-restart) | Fully idle system reference (no Isaac Sim at all) | Reference point for "Idle" row across all conditions |
| `trial_R{n}_S-{tier}_rep{k}_resources.csv` | `resource_sampler.sh` | 1 Hz CPU%, RAM MiB, GPU util%, GPU mem MiB, timestamped | **Not yet auto-consumed by `analyze_trial.py`** — currently a manual/separate inspection; integration pending (see note below) |
| `trial_R{n}_S-{tier}_rep{k}_bag/` (`.db3` + `metadata.yaml`) | `ros2 bag record` | Raw `/tf`, `/scan`, `/clock`, `/chassis/odom` (+ tier-dependent sensor topics) messages | `analyze_trial.py` → RTF, sim/wall elapsed time, path length, message counts |
| `trial_R{n}_S-{tier}_rep{k}_navgoal.log` | `isaac_ros_navigation_goal` launch, via `tee` | Goal-generated / goal-accepted / "Result" timestamps | `analyze_trial.py` → goal count, per-goal duration (wall-clock); **not reliable for success/failure** |
| `trial_R{n}_S-{tier}_rep{k}_nav2.log` | `carter_navigation` (Nav2) launch, via `tee` | `[bt_navigator]: Goal succeeded` / `Goal failed` (authoritative outcome) | `analyze_trial.py` → true `n_succeeded`, `n_failed`, `success_rate` |
| `results_summary.csv` | `analyze_trial.py` (appended per trial) | One row per trial: all metrics above, tagged by `(robots, sensor_tier, rep)` | Final analysis / plotting scripts (Graphs 1-4, Tables 1-2) — **the master results file for the paper** |

> **Known gap:** `resources.csv` (CPU/RAM/GPU) is currently captured
> per trial but not yet folded into `results_summary.csv` by
> `analyze_trial.py`. Planned extension: add a `--resources` argument
> that computes mean/max CPU, RAM, and GPU utilization over the trial's
> active window and appends those columns to the same summary row.

### Naming convention reference

```
baseline_R{n}_S-{tier}_loaded-paused.txt
baseline_R{n}_S-{tier}_playing-idle.txt
trial_R{n}_S-{tier}_rep{k}_bag/
trial_R{n}_S-{tier}_rep{k}_resources.csv
trial_R{n}_S-{tier}_rep{k}_navgoal.log
trial_R{n}_S-{tier}_rep{k}_nav2.log
```
where `{n}` = robot count (1, 2, 3), `{tier}` = sensor tier (low,
medium, high), `{k}` = repetition number (1, 2, 3, ...).
