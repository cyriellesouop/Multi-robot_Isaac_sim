#!/bin/bash
# resource_sampler.sh
#
# Samples CPU, RAM, and GPU utilization/memory once per second and appends
# each reading as a CSV row. Designed to run in parallel with a trial
# (Isaac Sim + Nav2 + navigation goal) and be stopped with Ctrl+C once the
# trial finishes.
#
# Usage:
#   ./resource_sampler.sh <output_csv_path> [interval_seconds]
#
# Example:
#   ./resource_sampler.sh trial_R1_S-low_rep1_resources.csv 1
#
# Requires: sysstat (for mpstat) -> sudo apt install sysstat
#           nvidia-smi (comes with NVIDIA driver)

set -euo pipefail

OUTFILE="${1:-resources.csv}"
INTERVAL="${2:-1}"

# Check dependencies up front so failures are obvious immediately,
# not after the trial has already started.
command -v mpstat >/dev/null 2>&1 || { echo "ERROR: mpstat not found. Run: sudo apt install sysstat"; exit 1; }
command -v nvidia-smi >/dev/null 2>&1 || { echo "ERROR: nvidia-smi not found."; exit 1; }

# Write header only if the file doesn't already exist (allows resuming/appending
# safely if the script is restarted, without duplicating the header row).
if [ ! -f "$OUTFILE" ]; then
    echo "timestamp,cpu_used_percent,mem_used_mib,mem_total_mib,gpu_util_percent,gpu_mem_used_mib,gpu_mem_total_mib" > "$OUTFILE"
fi

echo "Sampling every ${INTERVAL}s -> ${OUTFILE}"
echo "Press Ctrl+C to stop."

trap 'echo ""; echo "Stopped. Wrote samples to ${OUTFILE}"; exit 0' INT TERM

while true; do
    TS=$(date +%s.%N)

    # CPU: %idle from mpstat, converted to %used. -P ALL/1 avg across all cores.
    CPU_IDLE=$(mpstat 1 1 | awk '/Average/ {print $NF}')
    CPU_USED=$(awk -v idle="$CPU_IDLE" 'BEGIN { printf "%.2f", 100 - idle }')

    # RAM in MiB
    read -r MEM_USED MEM_TOTAL <<< "$(free -m | awk '/^Mem:/ {print $3, $2}')"

    # GPU: utilization %, memory used/total in MiB (single GPU assumed —
    # if you have multiple GPUs, add --id=0 or parse per-line)
    GPU_LINE=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total \
                          --format=csv,noheader,nounits)
    GPU_UTIL=$(echo "$GPU_LINE" | cut -d',' -f1 | tr -d ' ')
    GPU_MEM_USED=$(echo "$GPU_LINE" | cut -d',' -f2 | tr -d ' ')
    GPU_MEM_TOTAL=$(echo "$GPU_LINE" | cut -d',' -f3 | tr -d ' ')

    echo "${TS},${CPU_USED},${MEM_USED},${MEM_TOTAL},${GPU_UTIL},${GPU_MEM_USED},${GPU_MEM_TOTAL}" >> "$OUTFILE"

    sleep "$INTERVAL"
done


# script to capture other metrics excepts the one reported by ROSBAG and this file

# Terminal: /tf Hz
#ros2 topic hz /tf > trial_R1_S-low_rep1_tf_hz.log 2>&1

# Terminal: /scan Hz
#ros2 topic hz /scan > trial_R1_S-low_rep1_scan_hz.log 2>&1

# Terminal: /tf bandwidth
#ros2 topic bw /tf > trial_R1_S-low_rep1_tf_bw.log 2>&1

# Terminal: /scan bandwidth
#ros2 topic bw /scan > trial_R1_S-low_rep1_scan_bw.log 2>&1
