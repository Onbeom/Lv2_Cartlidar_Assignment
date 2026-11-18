#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9
Y=/home/pa4/yolo-venv/bin/python
timeout 600 $Y infer_cp.py --data data/val --ckpt runs/d2/best.pt --box e3 --agnostic --out results_val/d3_e3.jsonl > results_val/d3_e3.log 2>&1
$Y score.py --data data/val --cp results_val/d3_e3.jsonl --out results_val/summary3.json > results_val/score3.log 2>&1
mkdir -p results_d3; timeout 600 $Y infer_cp.py --data data/eval --ckpt runs/d2/best.pt --box e3 --agnostic --out results_d3/cp_e3_agnostic.jsonl > results_d3/cp.log 2>&1
$Y score.py --data data/eval --tag results/tag.jsonl --cp results_d3/cp_e3_agnostic.jsonl --out results_d3/summary.json > results_d3/score.log 2>&1
timeout 600 $Y real_observe.py > results_real.log 2>&1
echo 끝 > results_d3/done.txt
