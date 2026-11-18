#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose/real_label; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9
P=/home/pa4/isaacsim-venv/bin/python; Y=/home/pa4/yolo-venv/bin/python; rm -f results/done_r1.txt; mkdir -p results runs
$Y make_real_data.py > results/make.log 2>&1
timeout 600 $Y ../e3_dets.py data/train > results/e3.log 2>&1
timeout 600 $Y ../infer_cp.py --data data/val --ckpt ../runs/d2/best.pt --box e3 --agnostic --out results/val_d2.jsonl > results/val_d2.log 2>&1
$Y score_real.py --data data/val --cp results/val_d2.jsonl --out results/val_d2.json > results/val_d2.score 2>&1
timeout 1500 $P train_real.py --data data/train --val data/val --out runs/r1 --resume ../runs/d2/best.pt --dets data/train/e3_dets.json > results/train_r1.log 2>&1
timeout 600 $Y ../infer_cp.py --data data/val --ckpt runs/r1/best.pt --box e3 --agnostic --out results/val_r1.jsonl > results/val_r1.log 2>&1
$Y score_real.py --data data/val --cp results/val_r1.jsonl --out results/val_r1.json > results/val_r1.score 2>&1
echo 끝 > results/done_r1.txt
