#!/bin/bash
set -euo pipefail
export TRITON_CACHE_DIR=/home/pa4/git/lv2_next_module/261030_1325/triton_cache
ROOT=/home/pa4/git/lv2_next_module
exec 9>/tmp/lv2_gpu.lock
flock -w 7200 9
timeout --signal=INT --kill-after=30s 1800 /home/pa4/isaacsim-venv/bin/python "$ROOT/261030_1325/check_gpu_filter.py" > "$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325/filter_gpu_v1.log" 2>&1
