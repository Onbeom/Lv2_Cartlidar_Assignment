#!/bin/bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT="$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325"
export PYTHONUNBUFFERED=1
exec 9>/tmp/lv2_gpu.lock
flock -w 7200 9
for category in cylinder cuboid; do
  timeout --signal=INT --kill-after=30s 1800 /home/pa4/isaacsim-venv/bin/python "$ROOT/261030_1325/train_centerpose.py" "$category" > "$OUT/train_centerpose_${category}.log" 2>&1
done
