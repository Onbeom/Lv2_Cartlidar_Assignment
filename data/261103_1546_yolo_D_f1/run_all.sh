#!/usr/bin/env bash
# YOLO 후보 D: PC 엔진 빌드 → 빌드(dev c27d100) → F0(D435 288장) → F1(B0′ 24회, mission_map). PC. C 때 run_all.sh 와 엔진만 다름.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261103_1546_yolo_D_f1
WT=$HOME/git/Lv2_Cartlidar_Assignment_devD; SRC=$WT/workspace/ros2_ws/src
ONNX=$ROOT/261102_1634_yolo_D/yolo_export/yolo11n_task4_D.onnx
ENGINE=$D/trt/yolo11n_task4_D_fp32_notf32.engine
J=$ROOT/261031_1718_r3port/b0/judge
set +u; source /opt/ros/lyrical/setup.bash; set -u; export ROS_DOMAIN_ID=131
cd $D
colcon --log-base log build --base-paths $SRC --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1; echo "build_rc=$?" > result.log
colcon --log-base log test --base-paths $SRC --packages-select perception_survey --build-base build --install-base install > test.log 2>&1; echo "survey_test_rc=$?" >> result.log
for t in build/perception/test_*; do [ -x "$t" ] && { "$t" --gtest_brief=1 > "$t.out" 2>&1; echo "$(basename $t) rc=$? $(grep -c '^\[       OK' $t.out 2>/dev/null) $(tail -1 $t.out)" >> result.log; }; done
exec 9>/tmp/lv2_gpu.lock; flock 9
/usr/src/tensorrt/bin/trtexec --onnx=$ONNX --noTF32 --saveEngine=$ENGINE --skipInference > trt/trtexec_build.log 2>&1; echo "trtexec_rc=$?" >> result.log
F=$D/f0
g++ -std=c++17 -O2 -o $F/trt_team $ROOT/261101_0040_yolopost/equiv/trt_team.cpp -I$SRC/perception/include \
  $D/build/perception/libperception_detector.a $(pkg-config --cflags --libs opencv4) -lnvinfer -L/usr/local/cuda/lib64 -lcudart || { echo f0_compile_fail >> result.log; exit 1; }
RENDERS=$(ls -d $J/render_*_T????_awb | sort)
for r in $RENDERS; do ls $r/w*_d*_L?/frame.png; done | sort > $F/images.txt
sha256sum $0 $F/eval_f0.py $ENGINE $ONNX > $D/source_sha256.txt
git -C $WT rev-parse HEAD HEAD^{tree} >> $D/source_sha256.txt
$F/trt_team $ENGINE $F/images.txt > $F/trt_team.jsonl 2> $F/trt_team.err || echo "trt_team_fail $?" >> result.log
python3 -I $F/eval_f0.py $F/trt_team.jsonl $RENDERS > $F/summary.json && echo f0_done >> result.log
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
export REPLAY_DOMAIN=132
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE +detector.class_names=[red_cylinder,green_cylinder,red_cuboid,green_cuboid]"
sha256sum $TOOLS/set_yaml_values.py $TOOLS/run_replay.sh $TOOLS/replay_b0.py >> $D/source_sha256.txt
for s in 101 102 103 104 105 106; do for T in 3000 6500; do
  P=$D/f1/replay_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $D/install/setup.bash -- \
    --render $B0/judge/render_${s}_T${T}_awb --mission-map 0.729 0.960 3.143774 > $P.log 2>&1 || true
  test -f $P/summary.json || echo "summary 없음: $P" >> result.log
done; done
flock -u 9
echo done > $D/done.txt
