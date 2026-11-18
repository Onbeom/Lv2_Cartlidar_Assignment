#!/bin/bash
# 위치 다시 찾은 뒤 주행 이어 가기 확인(모의, 판정 기준은 R1~R5 + 참고 G1~G5): K2 와 같은 강제 이동 + run_goals --loc_retry 2(사본 run_goals.py.before_loc_retry_261102).
# K2c(run_k2_retry.sh) 가 끝나면 시작, K3 와 같은 강제 이동. PC, 도메인 102 · GPU 잠금.
HERE=/home/pa4/git/lv2_next_module/261101_0016_isaac_live; N=/home/pa4/git/lv2_next_module/261031_1325_nav2
until [ -e $HERE/results/K2c_done.txt ]; do sleep 20; done
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json; O=$HERE/results/K3c_retry_v45
LA="localization:=carto carto_config:=carto_localization_v22.lua carto_odom_topic:=/odometry/filtered turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=true ekf_params:=$N/config/ekf_sim_imu_icr.yaml rotation_center_tf:=true turn_floor_out_frame:=base_link objects_map:=$N/config/map_objects_fine_posts_walls.yaml move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] loc_monitor:=true loc_event_log:=$O/loc_events.jsonl"
EXTRA_TOPICS="/localization/status /localization/lock" bash $HERE/run_live_nav2.sh K3c_retry_v45 $N/config/goals_seed1_v26.json 1.0 $N/config/nav2_params_v45.yaml "$LA" \
  "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125 --kidnap_to 45 2.223 5.76 -0.652 --kidnap_still_s 2.5" \
  "--frame mission_map --base_offset_x -0.0786 --final_spin_tol 0.07 --final_spin_max 3 --loc_retry 2" > $HERE/results/K3c_retry_v45.console.log 2>&1
(source /opt/ros/lyrical/setup.bash; /usr/bin/python3 $N/test/evaluate_kidnap.py $O --k3 > $O/evaluate_kidnap.log 2>&1)
echo done > $HERE/results/K3c_done.txt
