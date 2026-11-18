#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose/real_label; export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9
P=/home/pa4/isaacsim-venv/bin/python; Y=/home/pa4/yolo-venv/bin/python; R=results; rm -f $R/done_r2.txt; mkdir -p render
for s in "train 2400 2001" "val 300 2002"; do set -- $s
  rm -rf render/$1; timeout 3000 $P render_real.py --out render/$1 --n $2 --seed $3 > $R/render_$1.log 2>&1
  $Y warp_sim.py render/$1 data/sim_$1 data/val/truth.json > $R/warp_$1.log 2>&1
  rm -rf render/$1
done
timeout 3000 $P train_real.py --data data/sim_train --val data/sim_val --out runs/r2a --resume ../runs/d2/best.pt --epochs 30 --lr 3e-4 > $R/train_r2a.log 2>&1
timeout 1800 $P train_real.py --data data/train --val data/val --out runs/r2 --resume runs/r2a/best.pt --dets data/train/e3_dets.json --sim_data data/sim_train --sim_n 800 --real_rep 4 --epochs 40 --lr 1e-4 > $R/train_r2.log 2>&1
for m in r2a r2; do timeout 600 $Y ../infer_cp.py --data data/val --ckpt runs/$m/best.pt --box e3 --agnostic --out $R/val_$m.jsonl > $R/val_$m.log 2>&1
  $Y repnp.py $R/val_$m.jsonl $R/val_${m}_s1.jsonl data/val --s 1 --rel .05; $Y score_real.py --data data/val --cp $R/val_${m}_s1.jsonl --out $R/val_${m}_s1.json > $R/val_${m}_s1.score 2>&1; done
echo 끝 > $R/done_r2.txt
