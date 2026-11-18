#!/usr/bin/env bash
# (모의, RL1 결정 11-11 07:4x (B)) r2e · r2d E1b 4 쌍씩 같은 seed 로 다시 (이번 회 값만 셈): 새 주 값 = 관절 0.02 rad + 나머지 최종 기준, 잡는 점 이동은 관찰
set -u
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
SUM="$OUT/e1_summary.txt"; say() { echo "$(date '+%F %T') $*" | tee -a "$SUM"; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$MAN"
say "E1 재집계 (B) 시작"
for x in "r2e N0_s777 e1a_r2e_N0_s777 0.0 777" "r2e N5_s777 e1a_r2e_N5_s777 0.5 777" "r2e N0_s778 e1a_r2e_N0_s778 0.0 778" "r2e N0_s779 e1a_r2e_N0_s779 0.0 779" \
         "r2d N0_s777 e1a_N0 0.0 777" "r2d N5_s777 e1a_N5 0.5 777" "r2d N0_s778 e1a_N0_s778 0.0 778" "r2d N0_s779 e1a_N0_s779 0.0 779"; do set -- $x
  L="$OUT/e1bJ_$1_$2.log"
  timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob $4 --lie_spot_virtual 1 --est --seconds 60 --lie_from_end "$OUT/$3.json" --seed $5 > "$L" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  say "E1bJ $1 $2 (seed $5)"
  grep -aE "Environment seed|E1b 끝 상태 넣음|최종 기준|항목별|E1 \(R0b'|\*\*E1 주 값|\*\*E1 새 주 값|관찰:" "$L" | sed 's/^/    /' | cut -c1-420 | tee -a "$SUM"
  grep -aiE -m3 -A6 "Traceback|out of memory" "$L" | tee -a "$SUM"
done
say "E1 재집계 (B) 끝"
