# 11-03 AMCL 비교 사본(run_reps_B.sh): launch 인자만 localization:=amcl(Cartographer 인자 뺌), 나머지 같음. params 는 config/nav2_params_v45_B_amcl.yaml 을 줌.
#!/bin/bash
# 11-03: launch 사본 nav2_mission_loc2.launch.py + costmap_map:=config/map_mission_nav_001.yaml(전역 costmap 두 정적 층 해상도 맞춤, 도착 순서 경쟁 없앰)
# 261102 B안 사본(261101_0016_isaac_live/run_reps.sh): 설계 45 판정 3 회(모의)를 B안 장면 · 설정으로. 바꾼 것: run_live_nav2_B.sh, 목표 config/goals_seed1_B2.json(11-03: L3 를 v26 자리로),
#   launch 인자에 x_icr:=-0.0658, run_goals 에 --base_offset_x -0.0658 --fp_rc -0.0892 0.3288 0.171, 시작 전 GPU 잠금이 빌 때까지 기다림(최대 6 시간, YOLO 학습).
# Isaac 비정상 종료(isaac_exit ≠ 0)면 폴더를 _TOOLFAIL_isaac_segv_tryN 으로 바꾸고 같은 seed 로 한 번 다시. 판정 불통과면 남은 seed 는 안 돌림.
#   사용: run_reps_B.sh <이름 앞> <params yaml> <seed...>   (PC)
PRE=$1; PARAMS=$2; shift 2
HERE=/home/pa4/git/lv2_next_module/261102_1646_nav2_B; N=/home/pa4/git/lv2_next_module/261031_1325_nav2
GOALS=$HERE/config/goals_seed1_B2.json
LA="localization:=amcl turn_floor:=true turn_floor_last:=true yaw_loop:=false turn_ff:=true turn_floor_w_min:=0.19 icr_odom:=true ekf_params:=$N/config/ekf_sim_imu_icr.yaml rotation_center_tf:=true turn_floor_out_frame:=base_link objects_map:=$N/config/map_objects_fine_posts_walls.yaml move_ff:=true move_ff_cmd:=[0.1,0.15,0.2,0.25,0.3,0.35,0.45,0.6,0.8] move_ff_act:=[0.0,0.0179,0.0496,0.0875,0.129,0.1719,0.261,0.3999,0.5824] x_icr:=-0.0658 costmap_map:=$HERE/config/map_mission_nav_001.yaml"
GA="--frame mission_map --base_offset_x -0.0658 --final_spin_tol 0.07 --final_spin_max 3 --fp_rc -0.0892 0.3288 0.171"
echo "wait_gpu_start $(date +%T)" >> $HERE/results/${PRE}_reps.txt
flock -w 21600 /tmp/lv2_gpu.lock true || { echo "GPU 잠금 6 시간 기다림 실패" >> $HERE/results/${PRE}_reps.txt; exit 1; }
echo "wait_gpu_end $(date +%T)" >> $HERE/results/${PRE}_reps.txt
for s in "$@"; do
  for try in 1 2; do
    NAME=${PRE}_s$s; R=$HERE/results/$NAME
    LAUNCH=$N/launch/nav2_mission_loc2.launch.py bash $HERE/run_live_nav2_B.sh $NAME $GOALS 1.0 $PARAMS "$LA" "--imu_topic /imu/data --occluder_mode analytic --imu_seed $s" "$GA" > $HERE/results/$NAME.console.log 2>&1
    if grep -q "^isaac_exit 0$" $R/exit_codes.txt; then break; fi
    k=1; while [ -e ${R}_TOOLFAIL_isaac_segv_try$k ]; do k=$((k+1)); done
    mv $R ${R}_TOOLFAIL_isaac_segv_try$k; mv $HERE/results/$NAME.console.log ${R}_TOOLFAIL_isaac_segv_try$k.console.log
  done
  echo "$NAME $(python3 -c "import json;d=json.load(open('$R/evaluate.json'));print('pass', d['pass'])" 2>&1)" >> $HERE/results/${PRE}_reps.txt
  grep -q "pass True" <(tail -1 $HERE/results/${PRE}_reps.txt) || break
done
echo done >> $HERE/results/${PRE}_reps.txt
