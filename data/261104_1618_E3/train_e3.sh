#!/bin/bash
# 후보 E3 학습(E2 와 같은 방식: ft1b_legacy6.pt 에서 80 epoch · batch 16 · 640 · seed 0 · amp 끔 · last.pt 고정, 자료만 E2 + 팔 근접 257장 × 3) → 경기장 판정 D · E2 · E3. PC, GPU 잠금.
set -uo pipefail
export YOLO_OFFLINE=true MPLCONFIGDIR=/tmp/lv2_plot_config
H=/home/pa4/git/lv2_next_module/261104_1618_E3; P=/home/pa4/yolo-venv/bin/python
Z=/home/pa4/git/lv2_next_module/261102_1141_tutor_yolo/zeroshot_b.py
exec 9>/tmp/lv2_gpu.lock; flock -w 21600 9 || { echo "GPU 잠금 실패"; exit 1; }
cd $H
timeout --signal=INT 10800 $P -c "
from ultralytics import YOLO
m = YOLO('/home/pa4/git/lv2_next_module/261029_1408/models/ft1b_legacy6.pt')
m.train(data='$H/E3_data/data.yaml', epochs=80, patience=0, batch=16, imgsz=640, workers=2, seed=0, amp=False, close_mosaic=10,
        project='$H/runs', name='E3', exist_ok=False, plots=True)
" > $H/train_E3.log 2>&1; echo "train_exit $?" > $H/exit_codes.txt
B_EV=/home/pa4/git/lv2_next_module/261030_1325/B_negative/images; T="/home/pa4/git/lv2_next_module/tutor's/yolov8-isaacsim-data"
F=("$B_EV/eval@ours" "$B_EV/sim_eval@ours" "$T/tagged-practice-20261029-154225/images/test")
$P $Z --device 0 --weights $H/runs/E3/weights/last.pt --out $H/eval_E3.json "${F[@]}" > $H/eval_E3.log 2>&1; echo "eval_E3_exit $?" >> $H/exit_codes.txt
A=/home/pa4/git/lv2_next_module/261104_1211_arena_eval
(cd $A && DETS_DEVICE=0 $P dets.py $H/runs/E3/weights/last.pt dets_E3.json arena_front arena_rear arena_arm > $H/dets_E3_arena.log 2>&1); echo "dets_E3_arena_exit $?" >> $H/exit_codes.txt
(cd $A && JUDGE_S0_FIXED=114 JUDGE_AGNOSTIC=arena_rear JUDGE_CAMS=arena_front,arena_rear,arena_arm JUDGE_OUT=judge_arena_E3.json $P judge.py D E2 E3 > $H/judge_arena_E3.log 2>&1); echo "judge_E3_exit $?" >> $H/exit_codes.txt
echo done > $H/done.txt
