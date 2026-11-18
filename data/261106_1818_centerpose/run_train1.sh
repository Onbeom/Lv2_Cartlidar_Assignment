#!/bin/bash
# 설계 1: crop 128 · 256, 40 epoch. 검증 자료로만 비교(평가 자료는 안 읽음)
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9
P=/home/pa4/isaacsim-venv/bin/python; Y=/home/pa4/yolo-venv/bin/python
timeout 3000 $P train_crop.py --data data/train --val data/val --out runs/s128 --size 128 --epochs 40 --batch 32 > runs_s128.log 2>&1
timeout 3600 $P train_crop.py --data data/train --val data/val --out runs/s256 --size 256 --epochs 40 --batch 12 > runs_s256.log 2>&1
python3 make_list.py data/val > results_val_list.txt; mkdir -p results_val; tagtool/puck_tool results_val_list.txt > results_val/tag.jsonl
for s in s128 s256; do for b in e3 oracle; do
  timeout 900 $Y infer_cp.py --data data/val --ckpt runs/$s/best.pt --box $b --out results_val/${s}_$b.jsonl > results_val/${s}_$b.log 2>&1
done; done
$Y score.py --data data/val --tag results_val/tag.jsonl --cp results_val/s128_e3.jsonl results_val/s128_oracle.jsonl results_val/s256_e3.jsonl results_val/s256_oracle.jsonl --out results_val/summary.json > results_val/score.log 2>&1
echo 끝 > results_val/done.txt
