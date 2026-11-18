#!/usr/bin/env bash
# (모의, 도구 확인) E1b 최종 기준 항목별 불통과: r2e N0 s777 · r2d F_N0 (seed 777) 끝 상태로 E1b 다시
set -u
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$MAN"
for x in "r2e $OUT/e1a_r2e_N0_s777.json" "r2d $OUT/e1a_N0.json"; do set -- $x
  timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_spot_virtual 1 --est --seconds 60 --lie_from_end "$2" --seed 777 > "$OUT/e1b_break_$1.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  echo "== $1"; grep -a "E1b \|E1 주 값" "$OUT/e1b_break_$1.log" | cut -c1-500
done
