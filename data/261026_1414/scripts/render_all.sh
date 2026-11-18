#!/bin/bash
# YOLO 자료 렌더 (PC, GPU). 다른 GPU 작업이 없을 때만 실행. 쓰는 법: render_all.sh <split...> (기본 train_a train_b val eval)
# 렌더 seed: train_a 20261101 · train_b 20261102 · val 20261201 · eval 20261301 (학습 · 검증 · 평가 따로)
set -e
cd $HOME/git/lv2_next_module
W=261026_1414/worlds; D=261026_1414/data
URDF=261022_2119/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf
R="$HOME/isaacsim-venv/bin/python Cartlidar/sim_render/render_yolo.py --urdf $URDF --out $D --/crashreporter/url="
TRAIN="--dist 0.3 1.6 --bearing 25 --jitter 2 --light-random --color-jitter 0.2"
mkdir -p 261026_1414/logs
for s in "${@:-train_a train_b val eval}"; do for x in $s; do
  if nvidia-smi --query-compute-apps=pid --format=csv,noheader | grep -q .; then echo "GPU 사용 중 — 멈춤"; nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv; exit 1; fi
  case $x in
    train_a) $R --worlds $W/train_a/seed_* --split train --part a --scenes 60 $TRAIN --seed 20261101 ;;
    train_b) $R --worlds $W/train_b/seed_* --split train --part b --scenes 60 $TRAIN --seed 20261102 ;;
    val)     $R --worlds $W/val/seed_* --split val --part 0 --scenes 50 $TRAIN --seed 20261201 ;;
    eval)    $R --worlds $W/eval/seed_* --split eval --part 0 --scenes 20 --dist 0.5 1.5 --bearing 15 --jitter 1 --lights 0 1 --seed 20261301 ;;
  esac > 261026_1414/logs/render_$x.log 2>&1
  tail -1 261026_1414/logs/render_$x.log
done; done
