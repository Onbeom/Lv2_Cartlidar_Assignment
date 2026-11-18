#!/bin/bash
# 후보 D 학습(B · C 와 같은 방식: ft1b_legacy6.pt 에서 80 epoch · batch 16 · 640 · seed 0 · amp 끔 · last.pt 고정) → B · C · D 를 같은 입력으로 비교. PC, GPU 잠금.
set -uo pipefail
export YOLO_OFFLINE=true MPLCONFIGDIR=/tmp/lv2_plot_config
H=/home/pa4/git/lv2_next_module/261102_1634_yolo_D; P=/home/pa4/yolo-venv/bin/python
Z=/home/pa4/git/lv2_next_module/261102_1141_tutor_yolo/zeroshot_b.py
exec 9>/tmp/lv2_gpu.lock; flock -w 7200 9 || { echo "GPU 잠금 실패"; exit 1; }
cd $H
timeout --signal=INT 10800 $P -c "
from ultralytics import YOLO
m = YOLO('/home/pa4/git/lv2_next_module/261029_1408/models/ft1b_legacy6.pt')
m.train(data='$H/D_real/data.yaml', epochs=80, patience=0, batch=16, imgsz=640, workers=2, seed=0, amp=False, close_mosaic=10,
        project='$H/runs', name='D_real', exist_ok=False, plots=True)
" > $H/train_D_real.log 2>&1; echo "train_exit $?" > $H/exit_codes.txt
B_EV=/home/pa4/git/lv2_next_module/261030_1325/B_negative/images; T="/home/pa4/git/lv2_next_module/tutor's/yolov8-isaacsim-data"
F=("$B_EV/eval@ours" "$B_EV/sim_eval@ours" "$T/tagged-practice-20261029-154225/images/test")
$P $Z --device 0 --weights $H/runs/D_real/weights/last.pt --out $H/eval_D.json "${F[@]}" > $H/eval_D.log 2>&1; echo "eval_D_exit $?" >> $H/exit_codes.txt
S=/home/pa4/git/lv2_next_module/261102_1330_yolo_close_draft; E=/home/pa4/git/lv2_next_module/261102_1625_close_eval
(cd $E && $P dets.py $H/runs/D_real/weights/last.pt dets_D.json $S/real_labels_close_front_C $S/real_labels_close_rear_C $S/real_labels_arm720_C > $H/dets_D.log 2>&1); echo "dets_D_exit $?" >> $H/exit_codes.txt
echo done > $H/done.txt
