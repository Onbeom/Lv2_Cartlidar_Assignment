#!/bin/bash
# 11-06 B4: dev EKF(odom_icr · ekf.yaml, 30c2d1f)로 K0 amcl 다시(Isaac 모의, B안, 11-03 run_k01_pkg.sh 와 같은 길 · 목표 7 개). PC.
#   A1~A3: 출발 0.734(11-03 판과 같은 자리, EKF 만 다름) · B1: 출발 0.69365(dev 기본 start_x, 장면 자리도 같이 옮김).
#   Isaac 139 면 _TOOLFAIL 로 바꾸고 3 번까지. 끝나면 results/done.txt
HERE=/home/pa4/git/lv2_next_module/261106_0946_b4_k0
OLD=/home/pa4/git/lv2_next_module/261103_1502_loc_pkg
EKF=/home/pa4/git/lv2_next_module/261106_0946_b4_k0/team_src/workspace/ros2_ws/src/bringup/config/ekf.yaml
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json
GOALS=/home/pa4/git/lv2_next_module/261102_1646_nav2_B/config/goals_seed1_B2.json
one() {   # $1 이름, $2 launch 더할 인자, $3 장면 더할 인자
  local O=$HERE/results/$1 try
  for try in 1 2 3; do
    LA="loc:=amcl turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=false use_ekf:=true ekf_params:=$EKF rotation_center_tf:=true turn_floor_out_frame:=base_link move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 loc_monitor:=true loc_event_log:=$O/loc_events.jsonl $2"
    EXTRA_TOPICS="/localization/status /localization/lock /amcl_pose" bash $HERE/run_live_b4.sh $1 $GOALS 1.0 $OLD/config/nav2_params_v45_B_map.yaml "$LA" \
      "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125 $3" \
      "--frame map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171" > $HERE/results/$1.console.log 2>&1
    if grep -q "^isaac_exit 139" $O/exit_codes.txt 2>/dev/null; then
      mv $O ${O}_TOOLFAIL_isaac_segv_try$try; mv $HERE/results/$1.console.log ${O}_TOOLFAIL_isaac_segv_try$try.console.log
      continue
    fi
    break
  done
  /usr/bin/python3 $HERE/judge.py > $HERE/results/judge.txt 2>&1
}
for n in ${@:-A1 A2 A3}; do one K0_dev_$n "start_x:=0.734" ""; done
one K0_dev_B1 "" "--start -1.54085 -2.2636 3.14159265"
date > $HERE/results/done.txt
