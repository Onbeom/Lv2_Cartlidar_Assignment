#!/bin/bash
# 11-03 A-2 묶음: 팀 localization 패키지(localization:=amcl, use_amcl_node true) + Nav2 설계 45 로 K0 판(강제 이동 없음)을 한 번 돌리며 영상 시각을 잰다.
# 인자는 261103_1502_loc_pkg/run_k01_pkg.sh 의 K0 amcl 과 같음(장면 · 목표 · EKF = 팀 ekf.yaml). Isaac 139 면 _TOOLFAIL 로 바꾸고 3 번까지. PC.
#   사용: run_a2_all.sh   끝나면 results/A2_done.txt
HERE=/home/pa4/git/lv2_next_module/261103_1909_cam_timing
LOCPKG=/home/pa4/git/lv2_next_module/261103_1502_loc_pkg
TEAM_EKF=/home/pa4/git/Lv2_Cartlidar_Assignment_loc/workspace/ros2_ws/src/bringup/config/ekf.yaml
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json
GOALS=/home/pa4/git/lv2_next_module/261102_1646_nav2_B/config/goals_seed1_B2.json
N=A2_amcl
for try in 1 2 3; do
  O=$HERE/results/$N
  LA="loc:=amcl turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=false use_ekf:=true ekf_params:=$TEAM_EKF rotation_center_tf:=true turn_floor_out_frame:=base_link move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 loc_monitor:=true loc_event_log:=$O/loc_events.jsonl"
  EXTRA_TOPICS="/localization/status /localization/lock /top_camera_front/infra1/camera_info /top_camera_front/infra2/camera_info" bash $HERE/run_a2.sh $N $GOALS 1.0 $LOCPKG/config/nav2_params_v45_B_map.yaml "$LA" \
    "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125" \
    "--frame map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171" > $HERE/results/$N.console.log 2>&1
  if grep -q "^isaac_exit 139" $O/exit_codes.txt 2>/dev/null; then
    mv $O ${O}_TOOLFAIL_isaac_segv_try$try; mv $HERE/results/$N.console.log ${O}_TOOLFAIL_isaac_segv_try$try.console.log
    continue
  fi
  break
done
date > $HERE/results/A2_done.txt
