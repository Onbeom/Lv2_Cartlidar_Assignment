#!/bin/bash
# D3b 대칭에 뒤집기 포함 지표, D6 같은 자료 · 같은 480장 + 물체마다 대칭 고르는 crop 학습
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9
P=/home/pa4/isaacsim-venv/bin/python; Y=/home/pa4/yolo-venv/bin/python
timeout 900 $P diag/d3b_first_model_flip.py > diag/d3b.log 2>&1
$P -I diag/convert_d435.py > diag/convert.log 2>&1
for c in cuboid cylinder; do D=diag/data_d435
  timeout 1500 $P train_crop.py --data $D/${c}_train480 --val $D/${c}_val80 --out diag/d6_runs/$c --size 128 --epochs 30 --batch 16 > diag/d6_$c.log 2>&1
  timeout 600 $Y infer_cp.py --data $D/${c}_eval --ckpt diag/d6_runs/$c/best.pt --box oracle --out diag/d6_runs/$c/oracle.jsonl > diag/d6_${c}_infer.log 2>&1
  $Y score.py --data $D/${c}_eval --cp diag/d6_runs/$c/oracle.jsonl --out diag/d6_runs/$c/summary.json > diag/d6_${c}_score.log 2>&1
done
timeout 600 /home/pa4/isaacsim-venv/bin/python render/render_cp.py --out render/toolcheck --n 12 --seed 9 > render/toolcheck.log 2>&1
