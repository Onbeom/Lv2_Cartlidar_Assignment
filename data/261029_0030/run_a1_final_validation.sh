#!/usr/bin/env bash
# 개인 인지 코드의 로컬 ROS 최종 확인. GPU/팀 저장소/네트워크 접근 없음.
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/a1_code_261028
cd "$ROOT"
grep -q '\[  PASSED  \] 119 tests.' "$OUT/gtest_v8.txt"
for data in inout grasp; do
  code=0
  bash "$ROOT/261028_1459/run_a1_design_replay.sh" v8 "$data" || code=$?
  test -f "$OUT/replay_${data}_v8/summary.json" || exit "$code"
  python3 "$ROOT/261029_0030/score_a1_perception.py" "$OUT/render_mission_$data" "$OUT/replay_${data}_v8" > "$OUT/score_${data}_v8.log"
done
code=0
CALIBRATED=1 bash "$ROOT/261028_1459/run_a1_design_replay.sh" v8 grasp || code=$?
test -f "$OUT/replay_grasp_v8_calibrated/summary.json" || exit "$code"
python3 "$ROOT/261029_0030/score_a1_perception.py" "$OUT/render_mission_grasp" "$OUT/replay_grasp_v8_calibrated" > "$OUT/score_grasp_v8_calibrated.log"
python3 - <<'PY'
from pathlib import Path
import json,hashlib
root=Path('/home/pa4/git/lv2_next_module');out=root/'Cartlidar/sim_render/results/a1_code_261028';pkg=root/'261028_1459/a1_work/a1_fix_src/workspace/ros2_ws/src/perception'
data={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(pkg.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
(out/'a1_source_v8_sha256.json').write_text(json.dumps(data,indent=2)+'\n')
summary={name:json.loads((out/name/'perception_summary.json').read_text()) for name in ['replay_inout_v8','replay_grasp_v8','replay_grasp_v8_calibrated']}
(out/'final_v8_perception_comparison.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:{x:v[x] for x in ['P1','P2','P3_other_frames','P3_hidden','P4','pass_observed_checks']} for k,v in summary.items()},ensure_ascii=False))
PY
