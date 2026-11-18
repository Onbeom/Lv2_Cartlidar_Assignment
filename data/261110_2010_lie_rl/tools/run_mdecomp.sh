#!/usr/bin/env bash
# 2026-11-11 MD: M3 원인 분해 (A단계만, seed 777, 옆 퍽 없음, 512) — 노드 규칙 · 고정 관측 · 인식 오차 하나씩 끔
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
S="$OUT/md_summary.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
BASE="--num_envs 512 --seed 777 --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011"
for v in "relOff --node_release 0" "obsOff --fix_obs 0" "measOff --meas 0"; do
  set -- $v; nm=$1; shift
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a.py" $BASE "$@" --dump "$OUT/md_${nm}_a.json" > "$OUT/md_${nm}_a.log" 2>&1
  echo "== $nm ($*)" | tee -a "$S"; grep -a "S6 까지 끝\|S4 끝 사유\|S4 접촉 없는\|Traceback" "$OUT/md_${nm}_a.log" | tee -a "$S"
  python3 - "$OUT/md_${nm}_a.json" <<'PY' | tee -a "$S"
import json, sys
R = json.load(open(sys.argv[1]))["rows"]
m3 = sum(1 for r in R if r["s4_end"] == "ok" and "S4" not in r["events"]["arm_box"] and "S4" not in r["events"]["puck_topple"])
print(f"  M3 (S4 끝 · 팔-상자 없음 · 넘어짐 없음) {m3}/{len(R)} ({100 * m3 / len(R):.1f} %)")
PY
done
echo "MD 끝 $(date '+%F %T')" | tee -a "$S"
