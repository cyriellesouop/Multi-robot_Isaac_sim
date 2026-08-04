#!/bin/bash
# resource_sampler.sh
#
# Samples CPU, RAM, and GPU utilization/memory once per second and appends
# each reading as a CSV row. Designed to run in parallel with a trial
# (Isaac Sim + Nav2 + navigation goal) and be stopped with Ctrl+C once the
# trial finishes.
#
# Usage:
#   ./resource_sampler.sh <output_csv_path> [interval_seconds] [--append]
#
# By default, each run OVERWRITES <output_csv_path> if it already exists,
# so restarting the script always starts a clean file scoped to just this
# run. Pass --append as a third argument to instead resume/append to an
# existing file (e.g. if you deliberately paused and are continuing the
# same trial's recording).
#
# Example:
#   ./resource_sampler.sh trial_R1_S-low_rep1_resources.csv 1
#   ./resource_sampler.sh trial_R1_S-low_rep1_resources.csv 1 --append
#
# Requires: sysstat (for mpstat) -> sudo apt install sysstat
#           nvidia-smi (comes with NVIDIA driver)

set -euo pipefail

OUTFILE="${1:-resources.csv}"
INTERVAL="${2:-1}"
MODE="${3:-}"

# Check dependencies up front so failures are obvious immediately,
# not after the trial has already started.
command -v mpstat >/dev/null 2>&1 || { echo "ERROR: mpstat not found. Run: sudo apt install sysstat"; exit 1; }
command -v nvidia-smi >/dev/null 2>&1 || { echo "ERROR: nvidia-smi not found."; exit 1; }

if [ "$MODE" = "--append" ]; then
    if [ ! -f "$OUTFILE" ]; then
        echo "timestamp,cpu_used_percent,mem_used_mib,mem_total_mib,gpu_util_percent,gpu_mem_used_mib,gpu_mem_total_mib" > "$OUTFILE"
    fi
    echo "Appending to existing file: ${OUTFILE}"
else
    if [ -f "$OUTFILE" ]; then
        echo "NOTE: ${OUTFILE} already existed and will be overwritten (default behavior)."
        echo "      Pass --append as a 3rd argument to resume/append instead."
    fi
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
