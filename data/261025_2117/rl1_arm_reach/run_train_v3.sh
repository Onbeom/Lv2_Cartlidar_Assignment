#!/usr/bin/env bash
# RL1 설계 v3(모의, 사용자 승인 2026-10-26):
#   1) 후보 S1·S2·S3를 seed 1·1024 env·200 iteration씩 학습(비교, 판정 아님)
#   2) 마지막 50 iteration 평균 위치 오차(Metrics/tcp_target/position_error)가 가장 작은 후보 선택 → results/v3_sweep.json
#   3) 그 후보로 seed 1·1000 iteration 본 학습 → 평가 J1~J5(tools/eval_rl1.py)
#   4) J1~J5를 모두 통과하면 seed 2·3도 학습·평가
# 기록: logs/rsl_rl/rl1_arm_reach/<날짜>_v3<후보>_*, 평가 results/eval/v3<후보>_seed<N>/, 로그 results/logs/
# 사용(PC): bash run_train_v3.sh
set -u
R=$(cd "$(dirname "$0")" && pwd)
LAB=$HOME/pa-course/IsaacLab-6.1
cd "$R"
task_of() { case $1 in s1) echo Waffle-RL1-ArmReachV3S1;; s2) echo Waffle-RL1-ArmReachV3S2;; s3) echo Waffle-RL1-ArmReachV3S3;; esac; }

for C in s1 s2 s3; do
  echo "[$(date +%H:%M:%S)] 비교 $C 시작"
  timeout 7200 uv run --project "$LAB" --frozen --extra isaacsim python train.py train \
    --task "$(task_of $C)" --num_envs 1024 --seed 1 --max_iterations 200 --run_name "v3${C}_sweep" \
    > "results/logs/train_v3${C}_sweep.log" 2>&1
  echo "[$(date +%H:%M:%S)] 비교 $C 끝 (exit $?)"
done

BEST=$(python3 - <<'EOF'
import json, re
res = {}
for c in ("s1", "s2", "s3"):
    txt = open(f"results/logs/train_v3{c}_sweep.log").read()
    v = [float(x) for x in re.findall(r"Metrics/tcp_target/position_error:\s*([-\d.e+]+)", txt)]
    res[c] = {"iterations_logged": len(v), "last50_mean_m": sum(v[-50:]) / max(1, len(v[-50:])) if v else None,
              "first_m": v[0] if v else None, "traceback": "Traceback" in txt}
ok = {c: r for c, r in res.items() if r["last50_mean_m"] is not None and r["iterations_logged"] >= 150 and not r["traceback"]}
best = min(ok, key=lambda c: ok[c]["last50_mean_m"]) if ok else None
json.dump({"candidates": res, "best": best, "rule": "마지막 50 iteration 평균 위치 오차 최소"},
          open("results/v3_sweep.json", "w"), ensure_ascii=False, indent=1)
print(best or "")
EOF
)
echo "[$(date +%H:%M:%S)] 비교 결과: $(cat results/v3_sweep.json | tr -d '\n' | tr -s ' ')"
if [ -z "$BEST" ]; then echo "선택할 후보 없음"; exit 1; fi

run_seed() {
  local S=$1
  echo "[$(date +%H:%M:%S)] v3$BEST seed $S 본 학습 시작"
  timeout 14400 uv run --project "$LAB" --frozen --extra isaacsim python train.py train \
    --task "$(task_of $BEST)" --num_envs 1024 --seed "$S" --max_iterations 1000 --run_name "v3${BEST}_seed$S" \
    > "results/logs/train_v3${BEST}_seed$S.log" 2>&1
  echo "[$(date +%H:%M:%S)] v3$BEST seed $S 학습 끝 (exit $?)"
  local CK
  CK=$(ls -d logs/rsl_rl/rl1_arm_reach/*_v3"${BEST}"_seed"$S" 2>/dev/null | tail -1)/model_999.pt
  if [ ! -f "$CK" ]; then echo "체크포인트 없음: $CK"; return 1; fi
  (cd "$LAB" && timeout 3600 uv run --frozen --extra isaacsim python "$R/tools/eval_rl1.py" --design "v3$BEST" \
    --checkpoint "$R/$CK" --num_envs 100 --n_targets 1000 --out "$R/results/eval/v3${BEST}_seed$S") \
    > "results/logs/eval_v3${BEST}_seed$S.log" 2>&1
  echo "[$(date +%H:%M:%S)] v3$BEST seed $S 평가 끝 (exit $?)"
  python3 -c "
import json, sys
v = json.load(open('results/eval/v3${BEST}_seed$S/summary.json'))['verdict']
print('판정', v)
sys.exit(0 if all(v.values()) else 1)"
}

if run_seed 1; then
  echo "seed 1이 J1~J5 통과 → seed 2·3 진행"
  run_seed 2
  run_seed 3
else
  echo "seed 1이 J1~J5 중 하나 이상 실패 → seed 2·3은 돌리지 않음"
fi
echo "[$(date +%H:%M:%S)] 모두 끝"
