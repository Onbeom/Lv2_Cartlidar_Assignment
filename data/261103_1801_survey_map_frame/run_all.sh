#!/usr/bin/env bash
# 측량 world_frame map(localization 패키지 꼴: map = 발제 → odom) 확인. F1(D) 과 같은 렌더 · 엔진, --mission-map 없이(map → odom 항등 = 발제). PC.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$(cd "$(dirname "$0")" && pwd)
WT=$HOME/git/Lv2_Cartlidar_Assignment_mapframe; SRC=$WT/workspace/ros2_ws/src
ENGINE=$ROOT/261103_1546_yolo_D_f1/trt/yolo11n_task4_D_fp32_notf32.engine
set +u; source /opt/ros/lyrical/setup.bash; set -u; export ROS_DOMAIN_ID=135
cd $D
colcon --log-base log build --base-paths $SRC --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1; echo "build_rc=$?" > result.log
colcon --log-base log test --base-paths $SRC --packages-select perception_survey --build-base build --install-base install > test.log 2>&1; echo "survey_test_rc=$?" >> result.log
colcon test-result --test-result-base build/perception_survey --all 2>&1 | tail -1 >> result.log
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
export REPLAY_DOMAIN=136
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE +detector.class_names=[red_cylinder,green_cylinder,red_cuboid,green_cuboid]"
sha256sum $0 $TOOLS/set_yaml_values.py $TOOLS/run_replay.sh $TOOLS/replay_b0.py $ENGINE > $D/source_sha256.txt
git -C $WT rev-parse HEAD >> $D/source_sha256.txt; git -C $WT diff | sha256sum >> $D/source_sha256.txt
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 101 102 103 104 105 106; do for T in 3000 6500; do
  P=$D/f1/replay_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $D/install/setup.bash -- \
    --render $B0/judge/render_${s}_T${T}_awb > $P.log 2>&1 || true
  test -f $P/summary.json || echo "summary 없음: $P" >> result.log
done; done
# 대조: 옛 설정(world_frame mission_map)을 새 꼴에 — mission_map frame 이 없어 관측을 못 넣을 것
for s in 101; do T=3000; P=$D/f1/control_mission_map_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY_SURVEY_SET="puck_survey.world_frame=mission_map" REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $D/install/setup.bash -- \
    --render $B0/judge/render_${s}_T${T}_awb > $P.log 2>&1 || true
done
flock -u 9
echo done > $D/done.txt
