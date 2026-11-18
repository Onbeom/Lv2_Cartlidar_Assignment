#!/usr/bin/env bash
# 2026-11-11 (미션 파트 요청) dev 5a69076 pallet.speed 0.8 -> 0.4 로 B단계 다시: M seed 777 끝 상태, σ 2 · 0 (sag · release 같음)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
sleep 180
exec 9>/tmp/lv2_gpu.lock; flock -w 18000 9 || exit 2
for sg in 2 0; do
  timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_b.py" --num_envs 100000 --scripted_pick $sg --grip_open 0.0042 --seed 2 --speed 0.4 --tilt_speed 0.4 --empty_speed 0.4 --row_speed 3:0.4 \
    --from_end "$OUT/m_nb0.0_s777_a.json" --out "$OUT/mbv04_sg${sg}_s777_b.json" > "$OUT/mbv04_sg${sg}_s777_b.log" 2>&1
done
echo "끝 $(date '+%F %T')"
