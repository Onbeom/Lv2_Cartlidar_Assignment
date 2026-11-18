#!/bin/bash
# 11-03 팀 저장소 localization 패키지 모의 확인(Isaac, B안): K0(강제 이동 없음) · K1(정지 중 0.20 m · 10° 강제 이동)을 carto → amcl 차례로.
# 261102_1646_nav2_B/run_k01_loc2_B.sh 사본에서 바꾼 것: launch = launch/test_stack.launch.py(패키지 + Nav2 설계 45, 위치 추정은 패키지만),
#   EKF = 팀 bringup/config/ekf.yaml(바퀴 vx + IMU yaw rate, icr_odom 없음), Nav2 params 프레임 mission_map → map(config/nav2_params_v45_B_map.yaml),
#   run_goals --frame map, 채점 tools/evaluate_kidnap_map.py. 장면 · 목표 · 설계 45 다른 인자는 같음. Isaac 139 면 _TOOLFAIL 로 바꾸고 3 번까지. PC.
#   사용: run_k01_pkg.sh [carto amcl]   끝나면 results/K01_pkg_done.txt
HERE=/home/pa4/git/lv2_next_module/261103_1502_loc_pkg
TEAM_EKF=/home/pa4/git/Lv2_Cartlidar_Assignment_loc/workspace/ros2_ws/src/bringup/config/ekf.yaml
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json
GOALS=/home/pa4/git/lv2_next_module/261102_1646_nav2_B/config/goals_seed1_B2.json
one() {   # $1 이름, $2 carto|amcl, $3 장면 추가 인자
  local O=$HERE/results/$1 try
  for try in 1 2 3; do
    LA="loc:=$2 turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=false use_ekf:=true ekf_params:=$TEAM_EKF rotation_center_tf:=true turn_floor_out_frame:=base_link move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 loc_monitor:=true loc_event_log:=$O/loc_events.jsonl"
    EXTRA_TOPICS="/localization/status /localization/lock /amcl_pose" bash $HERE/run_live_pkg.sh $1 $GOALS 1.0 $HERE/config/nav2_params_v45_B_map.yaml "$LA" \
      "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125 $3" \
      "--frame map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171" > $HERE/results/$1.console.log 2>&1
    if grep -q "^isaac_exit 139" $O/exit_codes.txt 2>/dev/null; then
      mv $O ${O}_TOOLFAIL_isaac_segv_try$try; mv $HERE/results/$1.console.log ${O}_TOOLFAIL_isaac_segv_try$try.console.log
      continue
    fi
    [ -n "$3" ] && (source /opt/ros/lyrical/setup.bash; /usr/bin/python3 $HERE/tools/evaluate_kidnap_map.py $O > $O/evaluate_kidnap.log 2>&1)
    break
  done
}
for L in ${@:-carto amcl}; do
  one K0_pkg_$L $L ""
  one K1_pkg_$L $L "--kidnap_by 45 0.1875 -0.0696 0.1745 --kidnap_still_s 2.5"
done
date > $HERE/results/K01_pkg_done.txt
