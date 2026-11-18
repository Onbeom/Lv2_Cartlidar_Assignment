#!/bin/bash
# 최종 판정 · 회귀 (모의). 앞 D435 ④ 는 노드 front_scales 와 같게 배율 1.0. 결과 json 은 EVAL_TAG=_pbbottom 으로 따로 저장 (옛 판정 기록 안 덮음)
P=/home/pa4/git/lv2_next_module/261112_1221_front_d435_cal/pb_bottom; T=$P/place_box_check; B=/home/pa4/git/lv2_next_module
export PB_TOOL=$T EVAL_TAG=_pbbottom
cd $P
python3 pb_run.py ../fk_dz/render $T --save final_fk.json > final_fk.txt
python3 pb_run.py occl/render $T --save final_occl.json > final_occl.txt
python3 $B/261112_0130_u3_render/eval_u3.py $B/261112_0600_box45/obl_judge > final_obl_eval.txt 2>&1
python3 $B/261112_0130_u3_render/eval_u3.py $B/261112_0130_u3_render/judge > final_u3_eval.txt 2>&1
PLACE_BOX_SCALES=1.0 python3 $B/261112_0600_box45/eval_d435.py $B/261112_0600_box45/judge > final_d435_eval.txt 2>&1
( source /opt/ros/lyrical/setup.bash; cd ~/git/Lv2_Cartlidar_Assignment_pbbottom/workspace/ros2_ws
  colcon test --packages-select perception > $P/colcon_test.log 2>&1
  colcon test-result --verbose --test-result-base build/perception > $P/test_result.log 2>&1 )
echo FINAL_DONE
