#!/bin/bash
set -euo pipefail
export YOLO_OFFLINE=true
export MPLCONFIGDIR=/tmp/lv2_plot_config
ROOT=/home/pa4/git/lv2_next_module
exec 9>/tmp/lv2_gpu.lock
flock -w 7200 9
timeout --signal=INT --kill-after=30s 1800 /home/pa4/yolo-venv/bin/python "$ROOT/261030_1325/evaluate_yolo.py" > "$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/yolo_evaluation.log" 2>&1
