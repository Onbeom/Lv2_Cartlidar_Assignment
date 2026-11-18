#!/usr/bin/env bash
# RL1 설계 v8 학습·평가(모의, 사용자 지시 2026-10-26: 실패하면 다음 설계로 바로). seed 1을 학습·평가하고 J1~J5를 모두 통과하면 seed 2·3을 이어서 돌린다(앞 설계들과 같은 규칙).
# 학습: num_envs 1024, 1000 iteration. 평가: 평가 표 1000개(tools/eval_rl1.py --design v8).
# 기록: logs/rsl_rl/rl1_arm_reach/<날짜>_v8_seed<N>/, 평가 results/eval/v8_seed<N>/summary.json, 로그 results/logs/
# 사용(PC): bash run_train_v8.sh
set -u
R=$(cd "$(dirname "$0")" && pwd)
LAB=$HOME/pa-course/IsaacLab-6.1
cd "$R"

run_seed() {
  local S=$1
  echo "[$(date +%H:%M:%S)] v8 seed $S 학습 시작"
  timeout 14400 uv run --project "$LAB" --frozen --extra isaacsim python train.py train \
    --task Waffle-RL1-ArmReachV8 --num_envs 1024 --seed "$S" --max_iterations 1000 --run_name "v8_seed$S" \
    > "results/logs/train_v8_seed$S.log" 2>&1
  echo "[$(date +%H:%M:%S)] v8 seed $S 학습 끝 (exit $?)"
  local CK
  CK=$(ls -d logs/rsl_rl/rl1_arm_reach/*_v8_seed"$S" 2>/dev/null | tail -1)/model_999.pt
  if [ ! -f "$CK" ]; then echo "체크포인트 없음: $CK"; return 1; fi
  (cd "$LAB" && timeout 3600 uv run --frozen --extra isaacsim python "$R/tools/eval_rl1.py" --design v8 \
    --checkpoint "$R/$CK" --num_envs 100 --n_targets 1000 --out "$R/results/eval/v8_seed$S") \
    > "results/logs/eval_v8_seed$S.log" 2>&1
  echo "[$(date +%H:%M:%S)] v8 seed $S 평가 끝 (exit $?)"
  python3 -c "
import json, sys
v = json.load(open('results/eval/v8_seed$S/summary.json'))['verdict']
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
