#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/a1_code_261028
flock /tmp/lv2_gpu.lock timeout --signal=TERM --kill-after=20s 1200 /home/pa4/isaacsim-venv/bin/python 261028_1459/render_a1_grasp.py --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_1" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_2" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_3" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_4" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_5" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_6" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --out "$OUT/render_mission_grasp" --scenes 1 --kinds grasp --seed 20261028 --load-plan "$OUT/mission_grasp_plan.json" --/crashreporter/url= > "$OUT/render_mission_grasp.log" 2>&1
# 도메인90은 같은 세션의 in/out 재생과 겹치지 않게 함.
END=$((SECONDS+1800))
while [ ! -s "$OUT/replay_mission_inout/summary.json" ]; do
  [ "$SECONDS" -lt "$END" ] || { echo 'in/out 재생 대기 초과'; exit 1; }
  sleep 3
done
REPLAY=replay_a1.py bash 261028_1459/run_replay_a1_private.sh "$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf" "$OUT/replay_mission_grasp" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" -- --render "$OUT/render_mission_grasp" > "$OUT/replay_mission_grasp.log" 2>&1
test -f "$OUT/replay_mission_grasp/summary.json"
