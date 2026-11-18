#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/integration_261030_0701/r4_d435
REPLAY=$ROOT/Cartlidar/sim_render/replay_r4.py timeout --signal=TERM --kill-after=20s 1800s bash "$ROOT/261030_0701/run_ros_r4.sh" "$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf" "$OUT" "$ROOT/261030_0701/install/setup.bash" --render "$ROOT/Cartlidar/sim_render/results/render_r4_1711_261027" d435 > "$OUT.log" 2>&1
test -s "$OUT/summary.json"
