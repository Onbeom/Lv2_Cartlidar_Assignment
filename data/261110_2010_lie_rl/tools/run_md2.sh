#!/usr/bin/env bash
# 2026-11-11 MD2: 고정 관측 칸별 (하나씩 살림), seed 777, 옆 퍽 없음, 512 — after_md (mo · mb0) 뒤 잠금 대기
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
S="$OUT/md2_summary.txt"
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9 || exit 2
BASE="--num_envs 512 --seed 777 --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011"
for lv in axis puck held box; do
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a.py" $BASE --live $lv --dump "$OUT/md2_${lv}_a.json" > "$OUT/md2_${lv}_a.log" 2>&1
  echo "== live $lv" | tee -a "$S"; grep -a "S4 끝 사유\|Traceback" "$OUT/md2_${lv}_a.log" | tee -a "$S"
  python3 - "$OUT/md2_${lv}_a.json" <<'PY' | tee -a "$S"
import json, sys
R = json.load(open(sys.argv[1]))["rows"]
m3 = sum(1 for r in R if r["s4_end"] == "ok" and "S4" not in r["events"]["arm_box"] and "S4" not in r["events"]["puck_topple"])
print(f"  M3 {m3}/{len(R)} ({100 * m3 / len(R):.1f} %)")
PY
done
echo "MD2 끝 $(date '+%F %T')" | tee -a "$S"
