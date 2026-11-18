#!/bin/bash
# 11-03: launch 사본 nav2_mission_loc2.launch.py + costmap_map:=config/map_mission_nav_001.yaml(전역 costmap 두 정적 층 해상도 맞춤, 도착 순서 경쟁 없앰)
# 위치 어긋남 감지 개선 확인(모의, B안): loc_monitor res_mode=interp_p75 · res_thr 0.034 · check_mode=pair(launch 사본 nav2_mission_loc2.launch.py)로 K0 · K1 다시.
# K0 = 강제 이동 없음(기준 R6: LOST 0 · SUSPECT 0), K1 = 정지 중 0.20 m · 10° 강제 이동(기준 R1~R5, evaluate_kidnap.py). 장면 인자는 K0_v45 · K1_v45 와 같음.
# K2e 가 끝난 뒤(results/K2e_done.txt) 차례로. Isaac 139 면 _TOOLFAIL_isaac_segv_try<n> 로 바꾸고 2 번까지 다시. PC.
HERE=/home/pa4/git/lv2_next_module/261102_1646_nav2_B; N=/home/pa4/git/lv2_next_module/261031_1325_nav2
LAY=/home/pa4/git/lv2_next_module/Cartlidar/sim_world/worlds/dice_mission_maze261030/seed_1/layout.json
while [ ! -e $HERE/results/K2e_done.txt ]; do sleep 60; done
grep -q "안 돎" $HERE/results/K2e_done.txt && { echo "앞 판정 불통과 — 안 돎" > $HERE/results/K01_loc2_done.txt; exit 0; }
one() {   # $1 이름, $2 장면 추가 인자
  local O=$HERE/results/$1 try
  for try in 1 2 3; do
    LA="localization:=carto carto_config:=carto_localization_v22.lua carto_odom_topic:=/odometry/filtered turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=true ekf_params:=$N/config/ekf_sim_imu_icr.yaml rotation_center_tf:=true turn_floor_out_frame:=base_link objects_map:=$N/config/map_objects_fine_posts_walls.yaml move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 costmap_map:=$HERE/config/map_mission_nav_001.yaml loc_monitor:=true loc_event_log:=$O/loc_events.jsonl loc_res_mode:=interp_p75 loc_res_thr:=0.034 loc_check_mode:=pair"
    LAUNCH=$N/launch/nav2_mission_loc2.launch.py EXTRA_TOPICS="/localization/status /localization/lock" bash $HERE/run_live_nav2_B.sh $1 $HERE/config/goals_seed1_B2.json 1.0 $HERE/config/nav2_params_v45_B.yaml "$LA" \
      "--imu_topic /imu/data --occluder_mode analytic --objects $LAY --legs 2.525 5.285 4.285 4.365 0.145 3.125 $2" \
      "--frame mission_map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171" > $HERE/results/$1.console.log 2>&1
    if grep -q "^isaac_exit 139" $O/exit_codes.txt 2>/dev/null; then
      mv $O ${O}_TOOLFAIL_isaac_segv_try$try; mv $HERE/results/$1.console.log ${O}_TOOLFAIL_isaac_segv_try$try.console.log
      continue
    fi
    [ -n "$2" ] && (source /opt/ros/lyrical/setup.bash; /usr/bin/python3 $N/test/evaluate_kidnap.py $O > $O/evaluate_kidnap.log 2>&1)
    break
  done
}
one K0_loc2_v45B ""
one K1_loc2_v45B "--kidnap_by 45 0.1875 -0.0696 0.1745 --kidnap_still_s 2.5"
echo done > $HERE/results/K01_loc2_done.txt
