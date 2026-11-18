#!/usr/bin/env bash
# (모의, RL1 11-11 03:4x) E1-F: E1b N0 · N5 다시 (대별 확인 플래그, --seed 777) -> E1-s: N0 seed 778 · 779 (E1a -> E1b) -> R0b'-s: R0b' --seed 778
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"; POL="$OUT/p1_r2d_policy.pt"
say() { echo "$(date '+%F %T') $*" | tee -a "$SUM"; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { say "잠금 대기 초과"; exit 2; }
cd "$MAN"
e1b() {  # $1 이름 $2 끝 상태 json $3 옆 퍽 확률 $4 seed
  timeout -k 30s -s INT 60m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob "$3" --lie_spot_virtual 1 --est --seconds 60 --lie_from_end "$2" --seed "$4" > "$OUT/e1b_$1.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  say "E1b $1 (seed $4)"
  grep -aE "Environment seed|E1b |\*\*E1|E1 끝 상태|끝 분류|\*\*성공" "$OUT/e1b_$1.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
  grep -aiE -m3 -A6 "Traceback|out of memory" "$OUT/e1b_$1.log" | tee -a "$SUM"
}
e1b F_N0 "$OUT/e1a_N0.json" 0.0 777
e1b F_N5 "$OUT/e1a_N5.json" 0.5 777
for sd in 778 779; do
  A="$OUT/e1a_N0_s$sd.json"
  timeout -k 30s -s INT 30m "$PY" lieupright_eval.py --policy "$POL" --num_envs 512 --seed $sd --nb_prob 0.0 --chain_phys 1 --dump_e1 "$A" > "$OUT/e1a_N0_s$sd.log" 2>&1
  say "E1a N0 seed $sd"
  grep -aE "\*\*R1|\*\*접촉 없는|E1 끝 상태|Environment seed" "$OUT/e1a_N0_s$sd.log" | sed 's/^/    /' | tee -a "$SUM"
  grep -aq Traceback "$OUT/e1a_N0_s$sd.log" && { grep -a -A8 Traceback "$OUT/e1a_N0_s$sd.log" | head -12 | tee -a "$SUM"; continue; }
  e1b N0_s$sd "$A" 0.0 $sd
done
S2="$OUT/r0_summary.txt"
echo "$(date '+%F %T') R0b'-s 시작 (R0b' 와 같음, --seed 778, 512)" | tee -a "$S2"
timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_spot_virtual 1 --est --seconds 60 --seed 778 > "$OUT/r0_R0b2_s778.log" 2>&1
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0b'-s 끝" | tee -a "$S2"
grep -aE "Environment seed|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N" "$OUT/r0_R0b2_s778.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$S2"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0b2_s778.log" | tee -a "$S2"
