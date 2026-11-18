#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$TOOLS/results/mission_261028
PLAN=$OUT/r3_plan.json
# 계획 완료 파일을 최대 30분 기다림. pgrep 사용하지 않음.
END=$((SECONDS + 1800))
while [ ! -s "$PLAN" ]; do
  if [ "$SECONDS" -ge "$END" ]; then echo '계획 대기 시간 초과'; exit 1; fi
  sleep 3
done
flock /tmp/lv2_gpu.lock /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_r3.py" --mission --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_1" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_2" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_3" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_4" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_5" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_6" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --out "$OUT/render_r3_gpu" --load-plan "$PLAN" --/crashreporter/url= > "$OUT/render_r3_gpu.log" 2>&1
python3 -I "$TOOLS/proto_r3/zbar_check.py" "$OUT/render_r3_gpu" > "$OUT/zbar_check_gpu.log" 2>&1
date -Iseconds > "$OUT/r3_done.txt"
