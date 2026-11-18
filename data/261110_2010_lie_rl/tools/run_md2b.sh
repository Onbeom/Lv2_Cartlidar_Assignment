#!/usr/bin/env bash
# 2026-11-11 MD2 덧붙임 (RL 261110 1 필수): 긴 축 = FK 갱신 (axis_fk), seed 777, 옆 퍽 없음, 512 — MD2 뒤 잠금 대기
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
S="$OUT/md2_summary.txt"
sleep 60  # MD2 가 먼저 잠금을 잡게
exec 9>/tmp/lv2_gpu.lock; flock -w 10800 9 || exit 2
BASE="--num_envs 512 --seed 777 --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011"
timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a.py" $BASE --live axis_fk --dump "$OUT/md2_axis_fk_a.json" > "$OUT/md2_axis_fk_a.log" 2>&1
echo "== live axis_fk" | tee -a "$S"; grep -a "S4 끝 사유\|Traceback" "$OUT/md2_axis_fk_a.log" | tee -a "$S"
python3 - "$OUT/md2_axis_fk_a.json" <<'PY' | tee -a "$S"
import json, sys
R = json.load(open(sys.argv[1]))["rows"]
m3 = sum(1 for r in R if r["s4_end"] == "ok" and "S4" not in r["events"]["arm_box"] and "S4" not in r["events"]["puck_topple"])
print(f"  M3 {m3}/{len(R)} ({100 * m3 / len(R):.1f} %)")
PY
echo "MD2b 끝 $(date '+%F %T')" | tee -a "$S"
