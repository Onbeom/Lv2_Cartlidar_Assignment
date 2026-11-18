#!/usr/bin/env bash
# 2026-11-11 B6 → r2g (RL 261110 1 조건부 승인 · 사용자 승인): r2e model_1997 에서 250 반복 → 중간 확인 (P1 64 대, B6 관측, scale 3.0; < 80 % 면 멈춤)
#   → 250 반복 더 → 판정: P1 B6 v 2 · 3 · 4 × seed 777 · 778 · 779 × 512 + M A단계 (노드 그대로, scale 3.0, 옆 퍽 없음) × 3 seed
#   bash run_b6.sh <회차 이름 (r2g / r2g2)> [시작 체크포인트]
set -u
NM="${1:-r2g}"
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
CK="${2:-$MAN/logs/rsl_rl/waffle_lieupright/2026-11-11_04-38-44/model_1997.pt}"
TASK=Waffle-LieUprightB6-Direct
S="$OUT/b6_summary.txt"
say() { echo "$(date '+%F %T') [$NM] $*" | tee -a "$S"; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { say "GPU 잠금 대기 실패"; exit 2; }
cd "$MAN"
train() { # 반복 수, 시작 체크포인트, 로그 이름 -> LAST 설정
  local TL="$OUT/b6_${NM}_$3_train.log"
  "$PY" train.py --task $TASK --num_envs 2048 --max_iterations $1 --viz none --checkpoint "$2" > "$TL" 2>&1 &
  local TP=$!
  python3 "$W/tools/p1_monitor.py" "$TL" "$TP" "$S" 100000 0.0 > "$OUT/b6_${NM}_$3_monitor.log" 2>&1
  wait $TP 2>/dev/null; sleep 5; pkill -9 -f "[t]rain.py --task $TASK" 2>/dev/null
  RUN=$(ls -d "$MAN"/logs/rsl_rl/waffle_lieupright/*/ | sort | tail -1)
  LAST=$(ls "$RUN"/model_*.pt 2>/dev/null | sort -t_ -k2 -n | tail -1)
  grep -aq "nan\|Traceback" "$TL" && grep -a -m3 "Traceback\|nan" "$TL" | tee -a "$S"
  say "학습 $3 끝: $LAST ($(grep -a 'Iteration time\|Total time' "$TL" | tail -2 | tr '\n' ' '))"
}
export_pol() { # 체크포인트 -> 정책 파일 $1
  timeout 300 "$PY" play.py --task $TASK --num_envs 1 --checkpoint "$LAST" --viz none > "$OUT/b6_${NM}_export.log" 2>&1 &
  local EP=$!; local RUN_=$(dirname "$LAST")
  until [ -f "$RUN_/exported/policy.pt" ] && [ "$RUN_/exported/policy.pt" -nt "$LAST" ] || ! kill -0 $EP 2>/dev/null; do sleep 2; done; sleep 3
  pkill -f "[p]lay.py --task $TASK" 2>/dev/null; wait $EP 2>/dev/null
  cp -f "$RUN_/exported/policy.pt" "$1" && say "정책 -> $1"
}
ev() { # 이름 대수 seed vmax
  timeout -k 30s -s INT 25m "$PY" lieupright_eval.py --cfg B6 --policy "$POL" --num_envs $2 --seed $3 --nb_prob 0.5 --vmax $4 --vmode scale > "$OUT/b6_${NM}_$1.log" 2>&1
  say "평가 $1: $(grep -a '접촉 없는 성공\|그중 닿음 [0-9]* =' "$OUT/b6_${NM}_$1.log" | tr '\n' ' ' | cut -c1-300)"
}
train 250 "$CK" a
POL="$OUT/p1_${NM}a_policy.pt"; export_pol "$POL"
ev mid64 64 777 3.0
P=$(grep -a '접촉 없는 성공' "$OUT/b6_${NM}_mid64.log" | grep -o '([0-9.]* %)' | tr -d '(%) ')
if [ -z "$P" ] || python3 -c "import sys; sys.exit(0 if float('$P') < 80 else 1)"; then say "중간 확인 ${P:-?} % < 80 % → 멈춤 (원인 보기)"; exit 3; fi
train 250 "$LAST" b
POL="$OUT/p1_${NM}_policy.pt"; export_pol "$POL"
for sd in 777 778 779; do for v in 2.0 3.0 4.0; do ev v${v}_s$sd 512 $sd $v; done; done
for sd in 777 778 779; do
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a.py" --policy "$POL" --num_envs 512 --seed $sd --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011 --dump "$OUT/b6_${NM}_m_s${sd}_a.json" > "$OUT/b6_${NM}_m_s${sd}_a.log" 2>&1
  python3 - "$OUT/b6_${NM}_m_s${sd}_a.json" <<'PY' | tee -a "$S"
import json, sys
R = json.load(open(sys.argv[1]))["rows"]
m3 = sum(1 for r in R if r["s4_end"] == "ok" and "S4" not in r["events"]["arm_box"] and "S4" not in r["events"]["puck_topple"])
print(f"  M3 {sys.argv[1].split('_')[-2]} {m3}/{len(R)} ({100 * m3 / len(R):.1f} %)")
PY
done
say "회차 끝"
