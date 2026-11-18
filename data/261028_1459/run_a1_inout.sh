#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/a1_code_261028
PLAN=$OUT/mission_inout_plan.json
END=$((SECONDS+1800))
while [ ! -s "$PLAN" ]; do
  [ "$SECONDS" -lt "$END" ] || { echo '자세 계획 대기 초과'; exit 1; }
  sleep 3
done
flock /tmp/lv2_gpu.lock /home/pa4/isaacsim-venv/bin/python 261028_1459/render_a1_view.py --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_1" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_2" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_3" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_4" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_5" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_6" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --out "$OUT/render_mission_inout" --scenes 2 --kinds in out --seed 20261028 --load-plan "$PLAN" --/crashreporter/url= > "$OUT/render_mission_inout.log" 2>&1
REPLAY=replay_a1.py bash 261028_1459/run_replay_a1_private.sh "$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf" "$OUT/replay_mission_inout" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" -- --render "$OUT/render_mission_inout" > "$OUT/replay_mission_inout.log" 2>&1
test -f "$OUT/replay_mission_inout/summary.json"
