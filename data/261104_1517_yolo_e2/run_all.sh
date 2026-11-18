#!/usr/bin/env bash
# YOLO E2 480 · 384 FP16 엔진(D → E2, 형식 같음): 빌드 → 단위 → 기동 4가지 → A1 3차 묶음 재생 → F1 24회 (PC)
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; W=$ROOT/261104_1517_yolo_e2
WT=$HOME/git/Lv2_Cartlidar_Assignment_e2; SRC=$WT/workspace/ros2_ws/src
A=$ROOT/261104_0112_arm_lock_yolo; T=$ROOT/261104_0056_tf_wait
E480=$W/trt/yolo11n_task4_E2_480_fp16.engine; E384=$W/trt/yolo11n_task4_E2_384_fp16.engine
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
R=$W/result.log
{ sha256sum $0 $E480 $E384 $A/run_replay_a1.sh $TOOLS/run_replay.sh $TOOLS/replay_b0.py $TOOLS/set_yaml_values.py; git -C $WT rev-parse HEAD; git -C $WT diff | sha256sum; } > $W/source_sha256.txt
set +u; source /opt/ros/lyrical/setup.bash; set -u
cd $W
colcon --log-base log build --base-paths $SRC --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1; echo "build_rc=$?" > $R
colcon --log-base log test --base-paths $SRC --packages-select perception perception_survey --build-base build --install-base install > test.log 2>&1; echo "test_rc=$?" >> $R
colcon test-result --test-result-base build --verbose > test_result.log 2>&1; tail -1 test_result.log >> $R

# 기동: 하나씩 차례로 (같은 컨테이너 이름이 섞이지 않게)
set +u; source $W/install/setup.bash; set -u
SHARE=$W/install/perception/share/perception/config
boot() {   # boot <이름> <set_yaml_values 인자...>
  local c=$1; shift; local O=$W/boot/$c; mkdir -p $O; cp $SHARE/*.yaml $O/
  [ $# -gt 0 ] && python3 -I $TOOLS/set_yaml_values.py $O/perception.yaml "$@"
  ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout -s INT 25 ros2 launch perception perception.launch.py config_dir:=$O > $W/boot/$c.log 2>&1
  echo "boot $c: $(grep -o '검출기: .*' $W/boot/$c.log | head -1 | cut -c1-400) | WARN·FATAL arm: $(grep -E 'arm.engine_path' $W/boot/$c.log | grep -o '\(WARN\|FATAL\).*' | head -1)" >> $R
  sleep 5
}
exec 9>/tmp/lv2_gpu.lock; flock 9
boot both detector.engine_path=$E480 arm.engine_path=$E384
boot shared detector.engine_path=$E480 'arm.engine_path=""'
boot arm_missing detector.engine_path=$E480 arm.engine_path=/nonexistent/arm.engine
boot no_engine arm.engine_path=$E384 detector.engine_path=/nonexistent/main.engine
flock -u 9

# A1 3차 묶음 (시드 20261106 12 장면) 재생 · 채점
OUT=$W/a1_e2
A1_SET="detector.engine_path=$E480 arm.engine_path=$E384" A1_DOMAIN=94 A1_ARM=main LOCK_LOG=1 \
  flock /tmp/lv2_gpu.lock env REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s \
  bash $A/run_replay_a1.sh $URDF $OUT $W/install/setup.bash --render $A/judge_set3 --settle 0.2 > $OUT.log 2>&1; echo "a1 replay_rc=$?" >> $R
test -s $OUT/summary.json && python3 $ROOT/261029_a1_new_model/score_perception.py $A/judge_set3 $OUT > ${OUT}_score.log 2>&1; echo "a1 score_rc=$?" >> $R

# F1 24회 (B0′, D435 480 · 팔 384)
export REPLAY_DOMAIN=138
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$E480 arm.engine_path=$E384"
mkdir -p $W/f1
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 101 102 103 104 105 106; do for TT in 3000 6500; do
  P=$W/f1/replay_${s}_T$TT; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $W/install/setup.bash -- \
    --render $B0/judge/render_${s}_T${TT}_awb > $P.log 2>&1 || true
  test -f $P/summary.json || echo "summary 없음: $P" >> $R
done; done
flock -u 9
echo done > $W/done.txt
