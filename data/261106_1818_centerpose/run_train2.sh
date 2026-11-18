#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9
P=/home/pa4/isaacsim-venv/bin/python; Y=/home/pa4/yolo-venv/bin/python; rm -f results_val/done2.txt
timeout 600 $Y e3_dets.py data/train > e3_dets.log 2>&1
timeout 3000 $P train_crop.py --data data/train --val data/val --out runs/d2 --size 256 --epochs 60 --batch 12 --lr 3e-4 --resume runs/s256/best.pt --dets data/train/e3_dets.json > runs_d2.log 2>&1
timeout 900 $Y infer_cp.py --data data/val --ckpt runs/d2/best.pt --box e3 --out results_val/d2_e3.jsonl > results_val/d2_e3.log 2>&1
timeout 900 $Y infer_cp.py --data data/val --ckpt runs/d2/best.pt --box e3 --tta --out results_val/d2_e3_tta.jsonl > results_val/d2_e3_tta.log 2>&1
$Y score.py --data data/val --tag results_val/tag.jsonl --cp results_val/d2_e3.jsonl results_val/d2_e3_tta.jsonl --out results_val/summary2.json > results_val/score2.log 2>&1
echo 끝 > results_val/done2.txt
