#!/usr/bin/env bash
# (모의, RL1 승인 11-11 07:0x) E1 을 r2e 로: r2d E1 (E1-F N0 · N5 seed 777, E1-s N0 seed 778 · 779) 과 같은 옵션 · seed, 바뀌는 것은 정책 하나
#   + E1a 옆 퍽 판정만 새 판정 (--nb_world 1) = r2d E1 과 다른 점. E1b (chain45) 최종 기준은 그대로.
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"; POL="$OUT/p1_r2e_policy.pt"
say() { echo "$(date '+%F %T') $*" | tee -a "$SUM"; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { say "잠금 대기 초과"; exit 2; }
cd "$MAN"
pair() {  # $1 이름 $2 옆 퍽 확률 $3 seed
  local A="$OUT/e1a_r2e_$1.json"
  timeout -k 30s -s INT 30m "$PY" lieupright_eval.py --policy "$POL" --num_envs 512 --seed $3 --nb_prob $2 --chain_phys 1 --nb_world 1 --dump_e1 "$A" > "$OUT/e1a_r2e_$1.log" 2>&1
  say "E1a r2e $1 (seed $3, 옆 퍽 $2, --nb_world 1)"
  grep -aE "\*\*R1|\*\*접촉 없는|E1 끝 상태|Environment seed|\[Phys" "$OUT/e1a_r2e_$1.log" | sed 's/^/    /' | tee -a "$SUM"
  grep -aq Traceback "$OUT/e1a_r2e_$1.log" && { grep -a -A8 Traceback "$OUT/e1a_r2e_$1.log" | head -12 | tee -a "$SUM"; return 1; }
  timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob $2 --lie_spot_virtual 1 --est --seconds 60 --lie_from_end "$A" --seed $3 > "$OUT/e1b_r2e_$1.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  say "E1b r2e $1 (seed $3)"
  grep -aE "Environment seed|\[Phys|E1b |\*\*E1|E1 끝 상태|끝 분류|\*\*성공" "$OUT/e1b_r2e_$1.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
  echo "    n5 넘김 $(grep -ac '45° 정책으로 다시 집기' "$OUT/e1b_r2e_$1.log") 대" | tee -a "$SUM"
  grep -aiE -m3 -A6 "Traceback|out of memory" "$OUT/e1b_r2e_$1.log" | tee -a "$SUM"
}
say "E1 r2e 시작"
pair N0_s777 0.0 777
pair N5_s777 0.5 777
pair N0_s778 0.0 778
pair N0_s779 0.0 779
say "E1 r2e 끝"
