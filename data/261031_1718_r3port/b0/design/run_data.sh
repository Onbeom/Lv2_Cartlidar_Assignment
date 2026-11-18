#!/usr/bin/env bash
# B0′ 설계 자료(판정 아님): 설계 월드 dice_mission_maze261030 seed 1~6 (색온도는 월드마다 하나) 계획 → 렌더(조명 0 · 1) → AWB.
# 판정 월드 101~106 과 겹치지 않게(P6). 계획은 CPU 병렬, 렌더는 GPU 잠금 순서.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$ROOT/261031_1718_r3port/b0/design
WS=$ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030
PY=/home/pa4/isaacsim-venv/bin/python
mkdir -p "$OUT/plan"
for s in 1 2 3 4 5 6; do
  [ -f "$OUT/plan/seed_$s.json" ] || $PY "$TOOLS/render_b0.py" --worlds "$WS/seed_$s" --seed $((20261101 + s)) \
    --out "$OUT/plan/unused_$s" --only-plan --save-plan "$OUT/plan/seed_$s.json" > "$OUT/plan/seed_$s.log" 2>&1 &
done
wait
set -e
for s in 1 2 3 4 5 6; do
  T=$(python3 -c "import json;print(json.load(open('$WS/seed_$s/layout.json'))['color_temp_k'])" 2>/dev/null || true)
  [ -n "$T" ] || T=$(python3 - "$WS/seed_$s/layout.json" <<'PY'
import json,sys
def f(o):
    if isinstance(o,dict):
        for k,v in o.items():
            if k=='color_temp_k': return v
            r=f(v)
            if r is not None: return r
print(f(json.load(open(sys.argv[1]))))
PY
)
  R=$OUT/render_$s
  [ -f "$R/render_b0.json" ] || flock /tmp/lv2_gpu.lock timeout 1800 $PY "$TOOLS/render_b0.py" \
    --worlds "$WS/seed_$s" --load-plan "$OUT/plan/seed_$s.json" --out "$R" --/crashreporter/url= > "$R.log" 2>&1
  [ -f "$R"_awb/awb.json ] || python3 -I "$TOOLS/awb.py" --in "$R" --out "$R"_awb --temp "$T" > "$R.awb.log" 2>&1
done
date -Iseconds > "$OUT/data_done.txt"
