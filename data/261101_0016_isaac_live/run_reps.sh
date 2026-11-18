#!/bin/bash
# 반복 실행(설계 판정용 3 회, 모의): 장면 잡음 seed 마다 run_live_nav2.sh 를 차례로(도메인 · GPU 잠금 때문에 동시 불가).
# Isaac 비정상 종료(isaac_exit ≠ 0)면 폴더를 _TOOLFAIL_isaac_segv 로 바꾸고 같은 seed 로 한 번 다시. 판정 불통과(evaluate.json pass false)면 남은 seed 는 안 돌림.
#   사용: run_reps.sh <이름 앞> <params yaml> <seed...>   (PC)   — launch · 장면 · 목표 실행기 인자는 설계 40 다시(live_nav_v40_occfix)와 같음
PRE=$1; PARAMS=$2; shift 2
HERE=/home/pa4/git/lv2_next_module/261101_0016_isaac_live; N=/home/pa4/git/lv2_next_module/261031_1325_nav2
GOALS=$N/config/goals_seed1_v26.json
LA="localization:=carto carto_config:=carto_localization_v22.lua carto_odom_topic:=/odometry/filtered turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=true ekf_params:=$N/config/ekf_sim_imu_icr.yaml rotation_center_tf:=true turn_floor_out_frame:=base_link objects_map:=$N/config/map_objects_fine_posts_walls.yaml move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824]"
GA="--frame mission_map --base_offset_x -0.0786 --final_spin_tol 0.07 --final_spin_max 3"
for s in "$@"; do
  for try in 1 2; do
    NAME=${PRE}_s$s; R=$HERE/results/$NAME
    bash $HERE/run_live_nav2.sh $NAME $GOALS 1.0 $PARAMS "$LA" "--imu_topic /imu/data --occluder_mode analytic --imu_seed $s" "$GA" > $HERE/results/$NAME.console.log 2>&1
    if grep -q "^isaac_exit 0$" $R/exit_codes.txt; then break; fi
    k=1; while [ -e ${R}_TOOLFAIL_isaac_segv_try$k ]; do k=$((k+1)); done   # 앞 실행의 실패 폴더와 이름이 겹치지 않게
    mv $R ${R}_TOOLFAIL_isaac_segv_try$k; mv $HERE/results/$NAME.console.log ${R}_TOOLFAIL_isaac_segv_try$k.console.log
  done
  echo "$NAME $(python3 -c "import json;d=json.load(open('$R/evaluate.json'));print('pass', d['pass'])" 2>&1)" >> $HERE/results/${PRE}_reps.txt
  grep -q "pass True" <(tail -1 $HERE/results/${PRE}_reps.txt) || break
done
echo done >> $HERE/results/${PRE}_reps.txt
