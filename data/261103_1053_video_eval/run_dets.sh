#!/bin/bash
# B · C · D 검출을 CPU 로(GPU 는 Nav2 세션이 씀) → dets_<모델>.json
cd /home/pa4/git/lv2_next_module/261103_1053_video_eval
export CUDA_VISIBLE_DEVICES= YOLO_OFFLINE=true
P=/home/pa4/yolo-venv/bin/python; R=/home/pa4/git/lv2_next_module
for m in B:$R/261030_1325/prototype/yolo_models/yolo11n_task4_B.pt C:$R/261102_1304_yolo_C/yolo_export/yolo11n_task4_C.pt D:$R/261102_1634_yolo_D/yolo_export/yolo11n_task4_D.pt; do
  $P dets.py ${m#*:} dets_${m%%:*}.json video_front video_rear video_arm > dets_${m%%:*}.log 2>&1; echo "${m%%:*} $?" >> dets_exit.txt
done
echo done > dets_done.txt
