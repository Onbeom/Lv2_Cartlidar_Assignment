#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
DATA=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/validation_v12
OUT=$DATA/replay_isolated
test -s "$DATA/render_a1.json"
REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s bash "$ROOT/261029_a1_new_model/run_ros_domain92.sh" "$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf" "$OUT" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" --render "$DATA" > "$OUT.log" 2>&1
test -s "$OUT/summary.json"
python3 "$ROOT/261029_a1_new_model/score_perception.py" "$DATA" "$OUT" > "${OUT}_score.log"
