#!/usr/bin/env bash
# r2e 채택 판정 (r2e 사슬 pid $1 끝 뒤): 옆 퍽 닿음 3 seed 평균 <= 0.5 % & 접촉 없는 성공 >= 93.3 % 면 r2e 채택, 아니면 r2f (B5) 500 반복 + 3 seed
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
SUM="$OUT/p1_rounds.txt"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
judge() {  # $1 = 이름 (r2e / r2f) -> "닿음% 성공%"
python3 - "$OUT" "$1" <<'PY'
import re,sys
o,nm=sys.argv[1],sys.argv[2]; nb=[];sc=[]
for sd in (777,778,779):
    try: t=open(f"{o}/p1_{nm}v_s{sd}.log",errors="ignore").read()
    except OSError: continue
    m=re.search(r"그중 닿음 \d+ = ([\d.]+) %",t); k=re.search(r"접촉 없는 성공 \(팔-상자 접촉 없음\) \d+/\d+ \(([\d.]+) %\)",t)
    if m and k: nb.append(float(m.group(1))); sc.append(float(k.group(1)))
print(f"{sum(nb)/len(nb):.2f} {sum(sc)/len(sc):.2f}" if len(nb)==3 else "nan nan")
PY
}
nan_gate() {  # RL1 11-11: 3 seed 중 하나라도 nan 종료 > 0.5 % 면 판정 전에 멈추고 재요청
python3 - "$OUT" "$1" <<'PY'
import re,sys
o,nm=sys.argv[1],sys.argv[2]; bad=[]
for sd in (777,778,779):
    try: t=open(f"{o}/p1_{nm}v_s{sd}.log",errors="ignore").read()
    except OSError: continue
    m=re.search(r"종료 이유: .*?nan (\d+)",t)
    if m and int(m.group(1))/512*100 > 0.5: bad.append(f"seed {sd} nan {m.group(1)}/512")
print("; ".join(bad))
PY
}
G=$(nan_gate r2e); if [ -n "$G" ]; then echo "$(date '+%F %T') [r2e] nan 종료 0.5 % 넘음 ($G) -> 판정 전 멈춤, 원인 붙여 RL1 재요청" | tee -a "$SUM"; exit 3; fi
read NB SC <<< "$(judge r2e)"
if python3 -c "import sys; sys.exit(0 if '$NB'!='nan' and float('$NB')<=0.5 and float('$SC')>=93.3 else 1)"; then
  echo "$(date '+%F %T') [r2e] 채택 (옆 퍽 닿음 평균 $NB %, 접촉 없는 성공 평균 $SC %)" | tee -a "$SUM"; exit 0; fi
echo "$(date '+%F %T') [r2e] 채택 기준 못 넘음 (옆 퍽 닿음 평균 $NB %, 접촉 없는 성공 평균 $SC %) -> r2f (B5)" | tee -a "$SUM"
RUN=$(ls -d "$MAN"/logs/rsl_rl/waffle_lieupright/*/ | sort | tail -1); CK=$(ls "$RUN"/model_*.pt | sort -t_ -k2 -n | tail -1)
TASK=Waffle-LieUprightB5-Direct bash "$W/tools/run_p1_round.sh" r2f "$CK" 500
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
for sd in 777 778 779; do
  L="$OUT/p1_r2fv_s$sd.log"
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_r2f_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 > "$L" 2>&1
  echo "$(date '+%F %T') [r2f-v] seed $sd" | tee -a "$SUM"; grep -aE "\[LUEval\]" "$L" | grep -v "정책" | sed "s/^/    /" | tee -a "$SUM"
done
G=$(nan_gate r2f); if [ -n "$G" ]; then echo "$(date '+%F %T') [r2f] nan 종료 0.5 % 넘음 ($G) -> 판정 전 멈춤, 원인 붙여 RL1 재요청" | tee -a "$SUM"; exit 3; fi
read NB SC <<< "$(judge r2f)"
if python3 -c "import sys; sys.exit(0 if '$NB'!='nan' and float('$NB')<=0.5 and float('$SC')>=93.3 else 1)"; then
  echo "$(date '+%F %T') [r2f] 채택 (옆 퍽 닿음 평균 $NB %, 접촉 없는 성공 평균 $SC %)" | tee -a "$SUM"
else
  echo "$(date '+%F %T') [r2f] 채택 기준 못 넘음 (옆 퍽 닿음 평균 $NB %, 접촉 없는 성공 평균 $SC %) -> 2 회 불통과, 멈추고 계획 단계 회피안 재요청" | tee -a "$SUM"
fi
