#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$TOOLS/results/mission_261028
PLAN=$OUT/r3_retry_ik_plan.json
# 계획 완료 파일을 최대 30분 기다림. pgrep 사용하지 않음.
END=$((SECONDS + 1800))
while [ ! -s "$PLAN" ]; do
  if [ "$SECONDS" -ge "$END" ]; then echo '계획 대기 시간 초과'; exit 1; fi
  sleep 3
done
python3 - "$OUT" <<'MERGE'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);first=json.loads((root/'r3_final_plan.json').read_text());retry=json.loads((root/'r3_retry_ik_plan.json').read_text())
first['plan'] += retry['plan']
first['unreachable'] = retry['unreachable']
(root/'r3_complete_plan.json').write_text(json.dumps(first,indent=1,ensure_ascii=False))
print('최종 팔 자세',len(first['plan']),'실패 조건',len(first['unreachable']))
MERGE
PLAN=$OUT/r3_complete_plan.json
flock /tmp/lv2_gpu.lock /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_r3.py" --mission --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_1" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_2" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_3" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_4" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_5" "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_6" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --out "$OUT/render_r3_complete" --load-plan "$PLAN" --/crashreporter/url= > "$OUT/render_r3_complete.log" 2>&1
python3 -I "$TOOLS/proto_r3/zbar_check.py" "$OUT/render_r3_complete" > "$OUT/zbar_check_complete.log" 2>&1
date -Iseconds > "$OUT/r3_done.txt"
