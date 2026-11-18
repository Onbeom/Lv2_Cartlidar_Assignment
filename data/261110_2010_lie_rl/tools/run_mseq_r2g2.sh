#!/usr/bin/env bash
# 2026-11-11 밤 M2: r2g2 로 미션 순서 모의 다시 (주 값 M1″, 미션 파트 261110 3) (RL 261111 1, RL 261110 1 조건부 승인 · 미션 파트 노드 값: 보기 A, scale 3.0, release 30 mm + 중심 보정 0.011)
#   주 회차: 옆 퍽 없음 × seed 777 · 778 · 779 (512), 기록 회차: 옆 퍽 반반 × seed 777 (512). A (S0 ~ S6) → B (S7 · S8) → 합치기
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
S="$OUT/mseq2_summary.txt"; POLICY="$W/handoff/lieupright_r2g2.pt"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 대기 실패" | tee -a "$S"; exit 2; }
ARGS=()
for c in "0.0 777" "0.0 778" "0.0 779" "0.5 777"; do
  set -- $c; nb=$1; sd=$2; nm="m2_nb${nb}_s$sd"
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a.py" --policy "$POLICY" --num_envs 512 --seed $sd --nb_prob $nb --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011 --dump "$OUT/${nm}_a.json" > "$OUT/${nm}_a.log" 2>&1
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_b.py" --num_envs 100000 --scripted_pick 2 --grip_open 0.0042 --seed 2 --from_end "$OUT/${nm}_a.json" --out "$OUT/${nm}_b.json" > "$OUT/${nm}_b.log" 2>&1
  echo "== $nm $(date '+%T')" | tee -a "$S"
  grep -a "Traceback" "$OUT/${nm}_a.log" "$OUT/${nm}_b.log" | tee -a "$S"
  python3 "$MS/mseq_summary.py" "$OUT/${nm}_a.json" "$OUT/${nm}_b.json" | tee -a "$S"
done
echo "== 주 회차 합계 (옆 퍽 없음 3 seed)" | tee -a "$S"
python3 "$MS/mseq_summary.py" "$OUT/m2_nb0.0_s777_a.json" "$OUT/m2_nb0.0_s777_b.json" "$OUT/m2_nb0.0_s778_a.json" "$OUT/m2_nb0.0_s778_b.json" "$OUT/m2_nb0.0_s779_a.json" "$OUT/m2_nb0.0_s779_b.json" | tail -1 | tee -a "$S"
echo "M 끝 $(date '+%F %T')" | tee -a "$S"
