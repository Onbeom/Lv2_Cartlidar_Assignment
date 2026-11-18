#!/usr/bin/env bash
# RL1 학습(seed 1·2·3, num_envs 1024, 1000 iteration)과 seed별 평가(J1~J5)를 차례로 실행한다(모의).
# 기록: logs/rsl_rl/rl1_arm_reach/<날짜>_seed<N>/, 평가: results/eval/seed<N>/summary.json, 진행 로그: results/logs/
# 사용(PC): bash run_train_all.sh
set -u
R=$(cd "$(dirname "$0")" && pwd)
LAB=$HOME/pa-course/IsaacLab-6.1
cd "$R"
for S in 1 2 3; do
  echo "[$(date +%H:%M:%S)] seed $S 학습 시작"
  timeout 14400 uv run --project "$LAB" --frozen --extra isaacsim python train.py train \
    --task Waffle-RL1-ArmReach --num_envs 1024 --seed $S --max_iterations 1000 --run_name seed$S \
    > results/logs/train_seed$S.log 2>&1
  echo "[$(date +%H:%M:%S)] seed $S 학습 끝 (exit $?)"
  CK=$(ls -d logs/rsl_rl/rl1_arm_reach/*_seed$S 2>/dev/null | tail -1)/model_999.pt
  if [ ! -f "$CK" ]; then echo "체크포인트 없음: $CK"; continue; fi
  (cd "$LAB" && timeout 3600 uv run --frozen --extra isaacsim python "$R/tools/eval_rl1.py" \
    --checkpoint "$R/$CK" --num_envs 100 --n_targets 1000 --out "$R/results/eval/seed$S") \
    > results/logs/eval_seed$S.log 2>&1
  echo "[$(date +%H:%M:%S)] seed $S 평가 끝 (exit $?)"
  grep -o '"verdict": {[^}]*}' "results/eval/seed$S/summary.json" 2>/dev/null
done
echo "[$(date +%H:%M:%S)] 모두 끝"
