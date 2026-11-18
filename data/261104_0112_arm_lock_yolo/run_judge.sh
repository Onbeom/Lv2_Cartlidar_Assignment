#!/usr/bin/env bash
# 판정: 새 A1 묶음(plan.py 시드 20261104, 6 월드 × 조명 2 = 12 장면) 렌더 → 이 브랜치로 main · color 재생 · 채점 (PC)
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261104_0112_arm_lock_yolo; A=$ROOT/261029_a1_new_model
J=$D/judge_set; SEED=20261104
{ sha256sum $0 $A/plan.py $A/render.py $A/replay.py $A/score_perception.py $D/run_set.sh $D/run_replay_a1.sh; git -C $HOME/git/Lv2_Cartlidar_Assignment_armlock rev-parse HEAD; git -C $HOME/git/Lv2_Cartlidar_Assignment_armlock diff | sha256sum; } > $D/source_sha256_judge.txt
P=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/plan_$SEED.json
[ -s $P ] || (cd $A && ~/isaacsim-venv/bin/python plan.py $SEED > $D/plan_$SEED.log 2>&1) || { echo "plan 실패" >> $D/judge_result.log; exit 1; }
mkdir -p $J
for world in 1 2 3 4 5 6; do
  [ -s $J/world_$world/render_a1.json ] && continue
  flock /tmp/lv2_gpu.lock timeout --signal=TERM --kill-after=20s 1800s ~/isaacsim-venv/bin/python $A/render.py --plan $P --world-seed $world --out $J/world_$world --/crashreporter/url= > $J/world_$world.log 2>&1
  test -s $J/world_$world/render_a1.json || { echo "render 실패 world $world" >> $D/judge_result.log; exit 1; }
done
python3 - "$J" <<'PY'
import json,sys
from pathlib import Path
out=Path(sys.argv[1]);allsc=[]
for w in range(1,7):
 p=out/f'world_{w}';d=json.loads((p/'render_a1.json').read_text())
 for s in d['scenes']:
  s['d435_images']={k:f'world_{w}/'+v for k,v in s['d435_images'].items()}
  for f in s['arm_frames']:f['image']=f'world_{w}/'+f['image']
  allsc.append(s)
d['scenes']=allsc;(out/'render_a1.json').write_text(json.dumps(d,indent=1)+'\n')
PY
for ARM in main color; do
  R=$J bash $D/run_set.sh j_$ARM $ARM
done
echo done > $D/judge_done.txt
