#!/bin/bash
# 최종 판정 · 회귀 (모의). 앞 D435 ④ 는 노드 front_scales 와 같게 배율 1.0. 결과 json 은 EVAL_TAG=_sc3 으로 따로 저장 (옛 판정 기록 안 덮음)
P=/home/pa4/git/lv2_next_module/261112_1221_front_d435_cal/pb_bottom; T=/home/pa4/git/lv2_next_module/261112_1221_front_d435_cal/pb_dev/place_box_check; B=/home/pa4/git/lv2_next_module
export PB_TOOL=$T EVAL_TAG=_sc3
export PLACE_BOX_SCALES=0.96,1.0,1.04   # (나) 팔 배율 3 개 (D435 ④ 줄은 1.0 그대로 덮어씀)
cd $P
python3 pb_run.py ../fk_dz/render $T --save sc3_fk.json > sc3_fk.txt
python3 pb_run.py occl/render $T --save sc3_occl.json > sc3_occl.txt
python3 $B/261112_0130_u3_render/eval_u3.py $B/261112_0600_box45/obl_judge > sc3_obl_eval.txt 2>&1
python3 $B/261112_0130_u3_render/eval_u3.py $B/261112_0130_u3_render/judge > sc3_u3_eval.txt 2>&1
PLACE_BOX_SCALES=1.0 python3 $B/261112_0600_box45/eval_d435.py $B/261112_0600_box45/judge > sc3_d435_eval.txt 2>&1

echo FINAL_DONE
