#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/new_model_261029/protocol_replay
REPLAY=$ROOT/261029_new_model_audit/check_perception_protocol.py bash "$ROOT/261028_1459/run_replay_a1_private.sh" "$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf" "$OUT" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" > "$ROOT/Cartlidar/sim_render/results/new_model_261029/perception_protocol.log" 2>&1
