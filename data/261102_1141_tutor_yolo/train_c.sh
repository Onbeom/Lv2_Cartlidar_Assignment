#!/bin/bash
# 후보 C 학습(B 와 같은 방식: ft1b_legacy6.pt 에서 80 epoch · batch 16 · 640 · seed 0 · amp 끔 · last.pt 고정) → B · C 를 같은 입력으로 비교 평가. PC, GPU 잠금.
set -uo pipefail
export YOLO_OFFLINE=true MPLCONFIGDIR=/tmp/lv2_plot_config
H=/home/pa4/git/lv2_next_module/261102_1141_tutor_yolo; P=/home/pa4/yolo-venv/bin/python
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9 || { echo "GPU 잠금 실패"; exit 1; }
cd $H
timeout --signal=INT 7200 $P -c "
from ultralytics import YOLO
m = YOLO('/home/pa4/git/lv2_next_module/261029_1408/models/ft1b_legacy6.pt')
m.train(data='$H/C_tutor/data.yaml', epochs=80, patience=0, batch=16, imgsz=640, workers=2, seed=0, amp=False, close_mosaic=10,
        project='$H/runs', name='C_tutor', exist_ok=False, plots=True)
" > $H/train_C_tutor.log 2>&1; echo "train_exit $?" > $H/exit_codes.txt
B_EV=/home/pa4/git/lv2_next_module/261030_1325/B_negative/images; T="/home/pa4/git/lv2_next_module/tutor's/yolov8-isaacsim-data"
F=("$B_EV/eval@ours" "$B_EV/sim_eval@ours" "$T/tagged-practice-20261029-154225/images/test" "$T/verification_ec886c9994fb/dataset/images/test" "$H/tutor_mission_verified/mission-batch-verified/images" "$H/tutor_mission_verified/mission-solved-verified/images")
$P $H/zeroshot_b.py --device 0 --out $H/eval_B.json "${F[@]}" > $H/eval_B.log 2>&1; echo "eval_B_exit $?" >> $H/exit_codes.txt
$P $H/zeroshot_b.py --device 0 --weights $H/runs/C_tutor/weights/last.pt --out $H/eval_C.json "${F[@]}" > $H/eval_C.log 2>&1; echo "eval_C_exit $?" >> $H/exit_codes.txt
echo done > $H/done.txt
