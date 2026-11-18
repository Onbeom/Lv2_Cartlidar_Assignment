#!/usr/bin/env bash
# 2026-11-11 (RL 261111 1) MD 뒤: ① 'D 밖 벌림' 참 퍽 중심 – D (M seed 777 과 같은 설정, 벌림 기록 사본) ② B 고정 집기 측정 오차 0 (M seed 777 끝 상태) — 칸 벽 손가락 접촉 원인 가르기
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; OUT="$W/results"; MS="$W/mission_seq"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9 || exit 2
timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_a_open.py" --num_envs 512 --seed 777 --nb_prob 0.0 --vmax 3.0 --vmode scale --view A --release_xy 0.03 --release_center_offset 0.011 --dump "$OUT/mo_s777_a.json" > "$OUT/mo_s777_a.log" 2>&1
timeout -k 30s -s INT 40m "$PY" "$MS/mission_seq_b.py" --num_envs 100000 --scripted_pick 0 --grip_open 0.0042 --seed 2 --from_end "$OUT/m_nb0.0_s777_a.json" --out "$OUT/mb0_s777_b.json" > "$OUT/mb0_s777_b.log" 2>&1
echo "끝 $(date '+%F %T')"
