#!/usr/bin/env bash
# E1 (모의, RL1 승인 11-11 02:0x): r2d 사슬 (pid $1) 끝 -> 정책 채택 (접촉 없는 성공 3 seed 평균 >= 93.3 % 면 r2d, 아니면 r2c)
#   -> E1a 8 대 (P1 평가 + chain45 물리, 끝 상태 저장) -> E1b 8 대 (chain45 --lie_from_end, R0b' 와 같은 옵션 줄) -> 통과면 512 (옆 퍽 0 · 0.5)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/e1_summary.txt"
say() { echo "$(date '+%F %T') $*" | tee -a "$SUM"; }
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
M=$(python3 - "$OUT" <<'PY'
import re,sys,glob
o=sys.argv[1]; v=[]
for sd in (777,778,779):
    try: t=open(f"{o}/p1_r2dv_s{sd}.log",errors="ignore").read()
    except OSError: continue
    m=re.search(r"접촉 없는 성공 \(팔-상자 접촉 없음\) (\d+)/(\d+)",t)
    if m: v.append(int(m.group(1))/int(m.group(2))*100)
print(f"{sum(v)/len(v):.2f}" if len(v)==3 else "nan")
PY
)
if [ "${E1_SMOKE:-0}" = 1 ]; then POL="$OUT/p1_r2c_policy.pt"; NMP=r2c; say "E1 코드 시험 (r2c, 8 대, r2d 전)"
elif python3 -c "import sys; sys.exit(0 if '$M'!='nan' and float('$M')>=93.3 else 1)"; then POL="$OUT/p1_r2d_policy.pt"; NMP=r2d; say "정책 채택: $NMP (r2d 접촉 없는 성공 3 seed 평균 $M %, 기준 r2c 93.3 %)"
else POL="$OUT/p1_r2c_policy.pt"; NMP=r2c; say "정책 채택: $NMP (r2d 접촉 없는 성공 3 seed 평균 $M %, 기준 r2c 93.3 %)"; fi
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { say "잠금 대기 초과"; exit 2; }
cd "$MAN"
run_pair() {  # $1 이름 $2 대 수 $3 옆 퍽 확률
  local A="$OUT/e1a_$1.json"
  timeout -k 30s -s INT 30m "$PY" lieupright_eval.py --policy "$POL" --num_envs "$2" --seed 777 --nb_prob "$3" --chain_phys 1 --dump_e1 "$A" > "$OUT/e1a_$1.log" 2>&1
  say "E1a $1 ($2 대, 옆 퍽 $3, 정책 $NMP)"
  grep -aE "\[LUEval\]|\[Phys" "$OUT/e1a_$1.log" | grep -v "정책 " | sed 's/^/    /' | tee -a "$SUM"
  grep -aq Traceback "$OUT/e1a_$1.log" && { grep -a -A8 Traceback "$OUT/e1a_$1.log" | head -14 | tee -a "$SUM"; return 1; }
  [ -f "$A" ] || { say "E1a 끝 상태 파일 없음"; return 1; }
  timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval "$2" --lie_regrasp --lie_layout mission --lie_nb_prob "$3" --lie_spot_virtual 1 --est --seconds 60 --lie_from_end "$A" > "$OUT/e1b_$1.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  say "E1b $1"
  grep -aE "\[Phys|E1b|E1 |\*\*E1|끝 분류|\*\*성공|팔-몸체 1 N" "$OUT/e1b_$1.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
  echo "    n5 넘김 $(grep -ac '45° 정책으로 다시 집기' "$OUT/e1b_$1.log") 대" | tee -a "$SUM"
  grep -aiE -m3 -A6 "Traceback|out of memory" "$OUT/e1b_$1.log" | tee -a "$SUM"
  grep -aqiE "Traceback|out of memory" "$OUT/e1b_$1.log" && return 1
  return 0
}
if [ "${E1_SMOKE:-0}" = 1 ]; then run_pair T8 8 0.0; say "E1 코드 시험 끝 (exit $?)"; exit 0; fi
run_pair S8 8 0.0 || { say "E1 8 대 시험 오류 -> 512 안 돌림"; exit 1; }
L8=$(grep -aoE "E1b 끝 상태 넣음 [0-9]+/8" "$OUT/e1b_S8.log" | grep -oE "[0-9]+/" | tr -d /)
C8=$(grep -aoE "최종 기준 통과 [0-9]+/[0-9]+" "$OUT/e1b_S8.log" | grep -oE "[0-9]+/[0-9]+")  # RL1 11-11 03:0x 최종 기준
H8=$(grep -ac '45° 정책으로 다시 집기' "$OUT/e1b_S8.log")
say "E1 8 대 판정: 넣음 ${L8:-0}, 최종 기준 ${C8:-없음}, n5 넘김 $H8 (기준: 최종 기준 = 넣은 대 전부, n5 넘김 6 이상)"
[ "${E1_S8_ONLY:-0}" = 1 ] && { say "E1 8 대 다시 시험 끝 (512 는 RL1 승인 뒤)"; exit 0; }
if [ "${C8:-x}" != "${L8:-0}/${L8:-0}" ] || [ "${H8:-0}" -lt 6 ]; then say "E1 8 대 시험 불통과 -> 512 안 돌림, RL1 재요청"; exit 1; fi
run_pair N0 512 0.0
run_pair N5 512 0.5
say "E1 끝"
