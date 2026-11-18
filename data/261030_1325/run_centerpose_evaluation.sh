#!/bin/bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9
timeout --signal=INT --kill-after=30s 1800 /home/pa4/isaacsim-venv/bin/python "$ROOT/261030_1325/evaluate_centerpose.py" > "$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/centerpose_evaluation.log" 2>&1
