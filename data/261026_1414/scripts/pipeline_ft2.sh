#!/bin/bash
# 실물 미세 조정 ft2(4클래스, 사용자 미리 승인 2026-10-27): GPU 가 2분 동안 비면 학습 → 실물 평가(ft1b 비교 · ft2)를 이어서 돌린다 (PC).
# 자료는 build_ft2.py 로 미리 만든 261026_1414/data_ft2(classes.txt 4클래스). 시작 가중치는 통과한 ft1b last.pt(6클래스 → 분류 머리만 4클래스로 새로).
# 학습 길이는 통과한 ft1b 와 같게(80 epoch 끝까지, patience 0, last.pt 평가). EPOCHS 환경 값으로 바꿀 수 있음.
# 판정: 평가 묶음 짝 지은 퍽 중 모양 ≥ 95 %(eval_real.py 기본), 빨강 · 초록은 eval_real.md 의 _red · _green 줄로 따로 봄.
# GPU 차례 약속은 pipeline_ft1b.sh 와 같음(/tmp/lv2_gpu.lock, 잠금 뒤 30초 재확인, rustdesk 는 GPU 작업으로 안 셈).
set -e
B=$HOME/git/lv2_next_module/261026_1414
EPOCHS=${EPOCHS:-80}
cd $HOME/git/lv2_next_module
[ -f $B/data_ft2/classes.txt ] || { echo "data_ft2 가 없음 → build_ft2.py 먼저"; exit 1; }
for d in $B/runs/ft2 $B/results/eval_real_ft2 $B/results/eval_real_ft2_ft1b; do
  [ -e $d ] && { echo "$d 가 이미 있음"; exit 1; }
done

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
echo "[$(date +%H:%M:%S)] GPU 비어 있음 → ft2 학습 시작($EPOCHS epoch)"
~/yolo-venv/bin/python Cartlidar/yolo/train_yolo.py --data $B/data_ft2 --project $B/runs --name ft2 \
  --model $B/runs/ft1b/weights/last.pt --epochs $EPOCHS --patience 0 > $B/logs/train_ft2.log 2>&1
echo "[$(date +%H:%M:%S)] 학습 끝 → 실물 평가(ft1b 비교, ft2)"
~/yolo-venv/bin/python Cartlidar/yolo/eval_real.py --model $B/runs/ft1b/weights/last.pt --data $B/data_ft2 --split eval \
  --out $B/results/eval_real_ft2_ft1b > $B/logs/eval_real_ft2_ft1b.log 2>&1
~/yolo-venv/bin/python Cartlidar/yolo/eval_real.py --model $B/runs/ft2/weights/last.pt --data $B/data_ft2 --split eval \
  --out $B/results/eval_real_ft2 > $B/logs/eval_real_ft2.log 2>&1
flock -u 9
echo "[$(date +%H:%M:%S)] 평가 끝, 잠금 풂"
cat $B/results/eval_real_ft2_ft1b/eval_real.md $B/results/eval_real_ft2/eval_real.md
