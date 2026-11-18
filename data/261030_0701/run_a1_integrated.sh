#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/integration_261030_0701/a1_design
code=0
REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s bash "$ROOT/261030_0701/run_ros_private.sh" "$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf" "$OUT" "$ROOT/261030_0701/install/setup.bash" --render "$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/design" > "$OUT.log" 2>&1 || code=$?
test -s "$OUT/summary.json" || exit "$code"
python3 "$ROOT/261029_a1_new_model/score_perception.py" "$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/design" "$OUT" > "${OUT}_score.log"
