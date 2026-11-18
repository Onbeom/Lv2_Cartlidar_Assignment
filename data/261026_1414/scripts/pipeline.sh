#!/bin/bash
# GPU 가 2분 동안 비면 렌더 → YOLO11n 학습 → R4 모양 판정(모의)을 이어서 돌린다 (PC).
# Isaac 렌더(최대 약 6.3 GB)는 다른 GPU 작업과 같이 돌리지 않는다.
# GPU 차례 약속(R3 판정 세션 제안 2026-10-26): 렌더 · 학습 · 판정을 /tmp/lv2_gpu.lock 잠금 안에서 돌린다.
#   잠금이 없는 RL 학습 때문에 잠금을 잡은 뒤에도 GPU 가 2분 비었는지 다시 보고, 아니면 잠금을 풀고 다시 기다린다.
set -e
B=$HOME/git/lv2_next_module/261026_1414
cd $HOME/git/lv2_next_module

wait_idle() {
  local free=0
  while [ $free -lt 4 ]; do
    if nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -q .; then free=0; else free=$((free+1)); fi
    sleep 30
  done
}

exec 9>/tmp/lv2_gpu.lock
while true; do
  wait_idle
  flock 9
  echo "[$(date +%H:%M:%S)] 잠금 잡음, GPU 다시 확인"
  if ! nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -q .; then
    sleep 30
    nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -q . || break
  fi
  echo "[$(date +%H:%M:%S)] GPU 사용 중 → 잠금 풀고 다시 기다림"
  flock -u 9
done
echo "[$(date +%H:%M:%S)] GPU 비어 있음 → 렌더 시작"
bash $B/scripts/render_all.sh train_a train_b val eval
echo "[$(date +%H:%M:%S)] 렌더 끝 → 학습 시작"
~/yolo-venv/bin/python Cartlidar/yolo/train_yolo.py --data $B/data --project $B/runs --name sim_v1 \
  --model $B/weights/yolo11n.pt > $B/logs/train_sim_v1.log 2>&1
echo "[$(date +%H:%M:%S)] 학습 끝 → R4 모양 판정"
~/yolo-venv/bin/python Cartlidar/yolo/eval_r4_shape.py --model $B/runs/sim_v1/weights/best.pt --data $B/data \
  --out $B/results/eval_r4_sim_v1 > $B/logs/eval_sim_v1.log 2>&1
flock -u 9
echo "[$(date +%H:%M:%S)] 판정 끝, 잠금 풂"
tail -12 $B/logs/eval_sim_v1.log
