#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$TOOLS/results/mission_261028
URDF=$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf
flock /tmp/lv2_gpu.lock /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_r12.py" --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_1" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --start-pose 0.734 0.950 180 --poses 20 --out "$OUT/render_r12_gpu" --/crashreporter/url= > "$OUT/render_r12_gpu.log" 2>&1
REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r1" "$OUT/build/install/setup.bash" -- --render "$OUT/render_r12_gpu" r1 > "$OUT/replay_r1.log" 2>&1
test -f "$OUT/replay_r1/summary.json"
REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r2" "$OUT/build/install/setup.bash" -- --render "$OUT/render_r12_gpu" r2 --mission --runs-per-group 20 --rate 1 > "$OUT/replay_r2.log" 2>&1
test -f "$OUT/replay_r2/summary.json"
date -Iseconds > "$OUT/signal_done.txt"
