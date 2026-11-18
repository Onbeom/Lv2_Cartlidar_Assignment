#!/bin/bash
# 후보 D 학습 이어 하기(11-02 16:4x 사용자: Nav2 세션이 GPU 를 먼저 씀 → epoch 6 에서 멈춤). last.pt 에서 resume → 평가는 train_d.sh 와 같음.
set -uo pipefail
export YOLO_OFFLINE=true MPLCONFIGDIR=/tmp/lv2_plot_config
H=/home/pa4/git/lv2_next_module/261102_1634_yolo_D; P=/home/pa4/yolo-venv/bin/python
Z=/home/pa4/git/lv2_next_module/261102_1141_tutor_yolo/zeroshot_b.py
exec 9>/tmp/lv2_gpu.lock; flock -w 21600 9 || { echo "GPU 잠금 실패"; exit 1; }
cd $H
timeout --signal=INT 10800 $P -c "
from ultralytics import YOLO
YOLO('$H/runs/D_real/weights/last.pt').train(resume=True)
" >> $H/train_D_real.log 2>&1; echo "resume_exit $?" >> $H/exit_codes.txt
B_EV=/home/pa4/git/lv2_next_module/261030_1325/B_negative/images; T="/home/pa4/git/lv2_next_module/tutor's/yolov8-isaacsim-data"
F=("$B_EV/eval@ours" "$B_EV/sim_eval@ours" "$T/tagged-practice-20261029-154225/images/test")
$P $Z --device 0 --weights $H/runs/D_real/weights/last.pt --out $H/eval_D.json "${F[@]}" > $H/eval_D.log 2>&1; echo "eval_D_exit $?" >> $H/exit_codes.txt
S=/home/pa4/git/lv2_next_module/261102_1330_yolo_close_draft; E=/home/pa4/git/lv2_next_module/261102_1625_close_eval
(cd $E && $P dets.py $H/runs/D_real/weights/last.pt dets_D.json $S/real_labels_close_front_C $S/real_labels_close_rear_C $S/real_labels_arm720_C > $H/dets_D.log 2>&1); echo "dets_D_exit $?" >> $H/exit_codes.txt
echo done > $H/done.txt
