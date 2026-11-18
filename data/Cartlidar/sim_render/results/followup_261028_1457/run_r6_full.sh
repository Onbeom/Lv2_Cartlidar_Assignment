#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/followup_261028_1457
python3 - "$OUT" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);design=json.loads((root/'r6_design3_replay/summary.json').read_text())
assert design['groups']==6 and design['pass'],design
plan=json.loads((root/'r6_judge_plan.json').read_text())['plan']
assert len({p['group_id'] for p in plan})==36
for seed in range(1,7):
    rows=[p for p in plan if p['seed']==seed]
    (root/f'r6_judge_plan_seed{seed}.json').write_text(json.dumps({'plan':rows},indent=1,ensure_ascii=False))
PY
for seed in 1 2 3 4 5 6; do
  valid=0
  for attempt in 1 2; do
    target="$OUT/r6_judge_batches/seed${seed}_try${attempt}"
    if [ -s "$target/render_r6.json" ]; then valid=1; break; fi
    if [ -e "$target" ]; then continue; fi
    echo "렌더 seed${seed} 시도${attempt}"
    if flock /tmp/lv2_gpu.lock timeout --kill-after=10s 600s /home/pa4/isaacsim-venv/bin/python "$ROOT/Cartlidar/sim_render/render_r6_judge.py" --worlds "$ROOT/Cartlidar/sim_world/worlds/dice_mission/seed_${seed}" --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission.json" --out "$target" --seed 20261102 --height 0.20 --load-plan "$OUT/r6_judge_plan_seed${seed}.json" --/crashreporter/url= > "$OUT/r6_judge_seed${seed}_try${attempt}.log" 2>&1; then
      if [ -s "$target/render_r6.json" ]; then valid=1; break; fi
    fi
  done
  if [ "$valid" != 1 ]; then echo "렌더 실패 seed${seed}"; exit 1; fi
  echo "$target" >> "$OUT/r6_judge_valid_batches.txt"
done
python3 - "$OUT" <<'PY'
import json,sys
from pathlib import Path
root=Path(sys.argv[1]);reports=[]
for part in dict.fromkeys((root/'r6_judge_valid_batches.txt').read_text().splitlines()):
    p=Path(part);meta=json.loads((p/'render_r6.json').read_text())
    for row in meta['poses']:
        row['images']={k:str((p/v).resolve()) for k,v in row['images'].items()}
    reports.append(meta)
merged=dict(reports[0]);merged['poses']=[p for m in reports for p in m['poses']];merged['source_batches']=(root/'r6_judge_valid_batches.txt').read_text().splitlines()
assert len(merged['poses'])==648,len(merged['poses'])
folder=root/'r6_judge_render';folder.mkdir(exist_ok=True);(folder/'render_r6.json').write_text(json.dumps(merged,indent=1,ensure_ascii=False))
print('완료 영상',len(merged['poses']))
PY
export ROS_LOG_DIR=$ROOT/261028_1457/ros_logs_r6_judge
export REPLAY=replay_r6_multiview.py
bash "$ROOT/Cartlidar/sim_render/run_replay_r6_multi.sh" "$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf" "$OUT/r6_judge_replay" "$ROOT/261028_1457/install/setup.bash" -- --render "$OUT/r6_judge_render" > "$OUT/r6_judge_replay.log" 2>&1
test -s "$OUT/r6_judge_replay/summary.json"
echo 'R6 본 판정 종료'
