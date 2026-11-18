#!/usr/bin/env bash
# F1 2차: 브랜치 fix/perception-survey-arm-vote 빌드 → 같은 판정 재생. PC.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261102_1308_yolo_C_f1; B=$D/build2
SRC=$HOME/git/Lv2_Cartlidar_Assignment_armvote/workspace/ros2_ws/src
ENGINE=$ROOT/261102_1304_yolo_C/trt_results/yolo11n_task4_C_fp32_notf32.engine
set +u; source /opt/ros/lyrical/setup.bash; set -u; export ROS_DOMAIN_ID=125
mkdir -p $B; cd $B; touch COLCON_IGNORE
colcon --log-base log build --base-paths $SRC --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1; echo "build_rc=$?" > result.log
colcon --log-base log test --base-paths $SRC --packages-select perception perception_survey --build-base build --install-base install > test.log 2>&1; echo "test_rc=$?" >> result.log
colcon test-result --test-result-base build --all > test_result.txt 2>&1; tail -1 test_result.txt >> result.log
git -C $HOME/git/Lv2_Cartlidar_Assignment_armvote diff origin/dev > $D/armvote.patch
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
export REPLAY_DOMAIN=128
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE +detector.class_names=[red_cylinder,green_cylinder,red_cuboid,green_cuboid]"
O=$D/f1_2; mkdir -p $O; sha256sum $0 $D/armvote.patch $ENGINE > $O/source_sha256.txt
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 101 102 103 104 105 106; do for T in 3000 6500; do
  P=$O/replay_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $B/install/setup.bash -- \
    --render $B0/judge/render_${s}_T${T}_awb --mission-map 0.729 0.960 3.143774 > $P.log 2>&1 || true
done; done
flock -u 9; echo done > $O/done.txt
