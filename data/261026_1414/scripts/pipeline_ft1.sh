#!/bin/bash
# 실물 미세 조정 시험 1(ft1): GPU 가 2분 동안 비면 학습 → 실물 평가(sim_v1 · ft1)를 이어서 돌린다 (PC, 사용자 승인 2026-10-26).
# 자료는 build_ft1.py 로 미리 만든 261026_1414/data_ft1. pipeline.sh 와 같은 GPU 차례 약속(/tmp/lv2_gpu.lock, 잠금 뒤 30초 재확인).
# rustdesk(원격 화면, GPU 약 140 MiB)는 GPU 작업으로 세지 않는다.
set -e
B=$HOME/git/lv2_next_module/261026_1414
cd $HOME/git/lv2_next_module

busy() { nvidia-smi --query-compute-apps=process_name --format=csv,noheader | grep -v -i rustdesk | grep -q .; }

wait_idle() {
  local free=0
  while [ $free -lt 4 ]; do
    if busy; then free=0; else free=$((free+1)); fi
    sleep 30
  done
}

exec 9>/tmp/lv2_gpu.lock
while true; do
  wait_idle
  flock 9
  echo "[$(date +%H:%M:%S)] 잠금 잡음, GPU 다시 확인"
  if ! busy; then
    sleep 30
    busy || break
  fi
  echo "[$(date +%H:%M:%S)] GPU 사용 중 → 잠금 풀고 다시 기다림"
  flock -u 9
done
echo "[$(date +%H:%M:%S)] GPU 비어 있음 → 미세 조정 학습 시작"
~/yolo-venv/bin/python Cartlidar/yolo/train_yolo.py --data $B/data_ft1 --project $B/runs --name ft1 \
  --model $B/runs/sim_v1/weights/best.pt --epochs 40 --patience 15 > $B/logs/train_ft1.log 2>&1
echo "[$(date +%H:%M:%S)] 학습 끝 → 실물 평가(sim_v1, ft1)"
for m in sim_v1 ft1; do
  ~/yolo-venv/bin/python Cartlidar/yolo/eval_real.py --model $B/runs/$m/weights/best.pt --data $B/data_ft1 --split eval \
    --out $B/results/eval_real_ft1_$m > $B/logs/eval_real_ft1_$m.log 2>&1
done
flock -u 9
echo "[$(date +%H:%M:%S)] 평가 끝, 잠금 풂"
tail -8 $B/results/eval_real_ft1_sim_v1/eval_real.md $B/results/eval_real_ft1_ft1/eval_real.md
