#!/bin/bash
# 11-03: launch 사본 nav2_mission_loc2.launch.py + costmap_map:=config/map_mission_nav_001.yaml(전역 costmap 두 정적 층 해상도 맞춤, 도착 순서 경쟁 없앰)
# K2 조건 재주행(모의, B안): K2d 와 같은 강제 이동(45 s 에 (2.2261, 1.5888, 1.16)) + 물체 · 기둥 3 개, run_goals --loc_retry 2 --escape 2 에 --loc_gate 2 를 더함
# (K2d 원인 = 다시 찾기 전 보낸 목표의 낡은 경로 추종 → OK 일 때만 보내기 + 다시 찾으면 다시 보내기). 기준: R1~R5(evaluate_kidnap.py) + G1 7/7 + G3 여유 > 0.
# 설계 45 B안 판정(run_reps_B.sh)이 3 회 모두 통과했을 때만 돎. Isaac 139 면 _TOOLFAIL_isaac_segv_try<n> 로 바꾸고 2 번까지 다시. PC.
HERE=/home/pa4/git/lv2_next_module/261102_1646_nav2_B; N=/home/pa4/git/lv2_next_module/261031_1325_nav2
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json
REPS=$HERE/results/live_nav_v45B3_reps.txt
while ! grep -q "^done" $REPS 2>/dev/null; do sleep 60; done
[ "$(grep -c "pass True" $REPS)" = 3 ] || { echo "설계 45 B안 판정이 3 회 통과가 아님 — K2e 안 돎" > $HERE/results/K2e_done.txt; exit 0; }
NAME=K2e_loc_gate_v45B; O=$HERE/results/$NAME
for try in 1 2 3; do
  LA="localization:=carto carto_config:=carto_localization_v22.lua carto_odom_topic:=/odometry/filtered turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=true ekf_params:=$N/config/ekf_sim_imu_icr.yaml rotation_center_tf:=true turn_floor_out_frame:=base_link objects_map:=$N/config/map_objects_fine_posts_walls.yaml move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 costmap_map:=$HERE/config/map_mission_nav_001.yaml loc_monitor:=true loc_event_log:=$O/loc_events.jsonl"
  EXTRA_TOPICS="/localization/status /localization/lock" LAUNCH=$N/launch/nav2_mission_loc2.launch.py bash $HERE/run_live_nav2_B.sh $NAME $HERE/config/goals_seed1_B2.json 1.0 $HERE/config/nav2_params_v45_B.yaml "$LA" \
    "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125 --kidnap_to 45 2.2261 1.5888 1.16 --kidnap_still_s 2.5" \
    "--frame mission_map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171 --loc_retry 2 --escape 2 --loc_gate 2" > $HERE/results/$NAME.console.log 2>&1
  if grep -q "^isaac_exit 139" $O/exit_codes.txt 2>/dev/null; then
    mv $O ${O}_TOOLFAIL_isaac_segv_try$try; mv $HERE/results/$NAME.console.log ${O}_TOOLFAIL_isaac_segv_try$try.console.log
    continue
  fi
  (source /opt/ros/lyrical/setup.bash; /usr/bin/python3 $N/test/evaluate_kidnap.py $O > $O/evaluate_kidnap.log 2>&1)
  break
done
echo done > $HERE/results/K2e_done.txt
