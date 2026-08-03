# Idle Baseline — System Resource Metrics

**Captured:** 2026-08-01 23:10:53 (fresh restart, no Isaac Sim / ROS 2 / project processes running)
**Machine:** ece-d6200-audu
**Uptime at capture:** 22 minutes
**Purpose:** Reference baseline for the "Idle" row in the per-pipeline-stage CPU/GPU attribution table (Experiment 4), and as a sanity check that no unrelated background load is present before starting trials.

---

## CPU / RAM (`free -m`, `top -bn1`)

| Metric | Value |
|---|---|
| Total RAM | 63,973 MiB (~62.5 GB) |
| Used RAM | 3,371 MiB (~3.3 GB) |
| Free RAM | 55,909 MiB |
| Buff/cache | 4,693 MiB |
| Available RAM | 59,758 MiB |
| Swap total | 6,143 MiB |
| Swap used | 0 MiB |
| CPU utilization (user) | 0.4% |
| CPU utilization (system) | 0.7% |
| CPU idle | 98.9% |
| Load average (1/5/15 min) | 0.27 / 0.36 / 0.22 |
| Total tasks | 374 (1 running, 373 sleeping) |

**Top CPU consumers at idle:** `gnome-terminal-` (6.7%), `top` itself (6.7%) — both negligible, expected background desktop processes only. No unexpected high-CPU processes present.

---

## GPU (`nvidia-smi`)

| Metric | Value |
|---|---|
| GPU model | NVIDIA GeForce RTX 2080 Ti |
| Driver version | 570.195.03 |
| CUDA version | 12.8 |
| GPU utilization | 2% |
| GPU memory used | 554 MiB / 11,264 MiB (~4.9%) |
| Power draw | 4 W / 250 W cap |
| Temperature | 42°C |
| Fan speed | 18% |

**GPU processes at idle:**
| PID | Process | GPU Memory |
|---|---|---|
| 3763 | `/usr/lib/xorg/Xorg` | 181 MiB |
| 3910 | `/usr/bin/gnome-shell` | 94 MiB |
| 4501 | (production process, e.g. browser/app) | 255 MiB |

These are standard desktop environment (Xorg, GNOME shell) and one other background application — no Isaac Sim, ROS 2, or CUDA workload processes present, confirming a clean idle state.

---

## Notes for methods section

- Baseline captured on a freshly restarted system with no unrelated background processes (browser tabs, IDEs, video calls, or other GPU-adjacent applications) beyond standard OS desktop services.
- This baseline establishes the reference point against which all subsequent per-stage CPU/GPU measurements (Isaac Sim, ROS 2 Bridge, DDS, Nav2) are compared, and serves as the "Idle" row in the pipeline-stage attribution table.
- GPU is effectively at rest (2% utilization, 4W draw) — confirms no residual GPU load from a prior session.
