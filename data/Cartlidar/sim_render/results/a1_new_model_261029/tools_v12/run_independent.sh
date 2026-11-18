#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/independent
mkdir -p "$OUT"
for world in 1 2 3 4 5 6; do
  flock /tmp/lv2_gpu.lock timeout --signal=TERM --kill-after=20s 1800s /home/pa4/isaacsim-venv/bin/python "$ROOT/261029_a1_new_model/render.py" --plan "$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/plan_20261129.json" --world-seed "$world" --out "$OUT/world_$world" --/crashreporter/url= > "$OUT/world_$world.log" 2>&1
  test -s "$OUT/world_$world/render_a1.json"
done
python3 - "$OUT" <<'PY'
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
code=0
REPLAY=$ROOT/261029_a1_new_model/replay.py bash "$ROOT/261028_1459/run_replay_a1_private.sh" "$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf" "$OUT/replay" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" --render "$OUT" > "$OUT/replay.log" 2>&1 || code=$?
test -s "$OUT/replay/summary.json" || exit "$code"
python3 "$ROOT/261029_a1_new_model/score_perception.py" "$OUT" "$OUT/replay" > "$OUT/perception_score.log"
