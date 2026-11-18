#!/bin/bash
# 학습 2400 · 검증 300 · 평가 500 장면 (시드 분리, 모의). 평가 자료는 학습 · 설계에 쓰지 않는다.
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9
P=/home/pa4/isaacsim-venv/bin/python
for s in "train 2400 1001" "val 300 1002" "eval 500 1003"; do set -- $s
  timeout 2400 $P render/render_cp.py --out data/$1 --n $2 --seed $3 > render/$1.log 2>&1
done
