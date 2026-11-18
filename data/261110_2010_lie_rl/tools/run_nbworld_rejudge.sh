#!/usr/bin/env bash
# 2026-11-11 (RL1 승인 뒤에만 실행): 옆 퍽 밀림을 월드 좌표로 고친 판정 (--nb_world 1) 으로 r2d · r2e · r2f 를 3 seed 다시 평가, 같은 채택 기준
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"; PY="$ISAACLAB_DIR/.venv/bin/python"
exec 9>/tmp/lv2_gpu.lock; flock -w 1800 9 || exit 2
cd "$MAN"
# RL1 조건 2: 같은 정책 · seed 로 옛 판정 (base, 접미 b) 과 새 판정 (월드, 접미 w) 을 둘 다 (원인별 · 차체 이동 출력 포함)
for nm in r2d r2e r2f; do for sd in 777 778 779; do for fr in b w; do
  L="$OUT/p1_${nm}${fr}_s$sd.log"; NW=0; [ $fr = w ] && NW=1
  timeout -k 30s -s INT 20m "$PY" lieupright_eval.py --policy "$OUT/p1_${nm}_policy.pt" --num_envs 512 --seed $sd --nb_prob 0.5 --nb_world $NW --nb_diag "$OUT/p1_${nm}${fr}_nbdiag_s$sd.json" > "$L" 2>&1
  echo "== $nm $sd $fr"; grep "R1 세워\|종료 이유\|그중 닿음 [0-9]* =\|접촉 없는 성공\|원인별\|차체 (전체" "$L"
done; done; done
python3 - "$OUT" <<'PY'
import re,sys
o=sys.argv[1]
for nm in [a+f for a in ("r2d","r2e","r2f") for f in ("b","w")]:
    nb=[];sc=[];nan=[]
    for sd in (777,778,779):
        t=open(f"{o}/p1_{nm}_s{sd}.log",errors="ignore").read()
        m=re.search(r"그중 닿음 \d+ = ([\d.]+) %",t); k=re.search(r"접촉 없는 성공 \(팔-상자 접촉 없음\) \d+/\d+ \(([\d.]+) %\)",t); z=re.search(r"종료 이유: .*?nan (\d+)",t)
        if m and k: nb.append(float(m.group(1))); sc.append(float(k.group(1)))
        if z: nan.append(int(z.group(1)))
    if len(nb)==3:
        ok = sum(nb)/3<=0.5 and sum(sc)/3>=93.3 and all(v/512*100<=0.5 for v in nan)
        print(f"[{nm}] 옆 퍽 닿음 {nb} 평균 {sum(nb)/3:.2f} %, 접촉 없는 성공 {sc} 평균 {sum(sc)/3:.2f} %, nan {nan} -> {'기준 통과' if ok else '기준 못 넘음'}")
    else: print(f"[{nm}] 값 없음")
PY
