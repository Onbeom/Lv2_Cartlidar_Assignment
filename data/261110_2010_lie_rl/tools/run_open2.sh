#!/usr/bin/env bash
# 2026-11-11 (RL 261110 1 제안) axis_fk 설정으로 벌림 벡터 기록 — 추정점 식 오프라인 비교 (MD2b 뒤 잠금 대기)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
sleep 120
exec 9>/tmp/lv2_gpu.lock; flock -w 14400 9 || exit 2
timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a_open2.py" --num_envs 512 --seed 777 --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011 --live axis_fk --dump "$OUT/mo2_s777_a.json" > "$OUT/mo2_s777_a.log" 2>&1
echo "끝 $(date '+%F %T')"
