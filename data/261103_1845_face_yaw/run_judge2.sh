#!/usr/bin/env bash
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261103_1845_face_yaw
SETUP=$ROOT/261103_1821_issue82_view/build_faceyaw/install/setup.bash
sha256sum $0 $D/render_face_yaw.py $D/node_face_yaw.py $D/run_node.sh > $D/source_sha256_judge2.txt
git -C $HOME/git/Lv2_Cartlidar_Assignment_faceyaw rev-parse HEAD >> $D/source_sha256_judge2.txt; git -C $HOME/git/Lv2_Cartlidar_Assignment_faceyaw diff | sha256sum >> $D/source_sha256_judge2.txt
(exec 9>/tmp/lv2_gpu.lock; flock 9; timeout 1800 ~/isaacsim-venv/bin/python $D/render_face_yaw.py --out $D/judge3 --n 40 --seed 3 > $D/judge3_render.log 2>&1; echo "render_rc=$?" >> $D/judge2_result.log)
bash $D/run_node.sh $SETUP $D/judge3 $D/judge3_node.json > $D/judge3_node.log 2>&1; tail -1 $D/judge3_node.log >> $D/judge2_result.log
# 회귀: 퍽 측량 F1 (엔진 D, world_frame map 은 dev 설정 = 아직 mission_map 이면 --mission-map 꼴로)
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
ENGINE=$ROOT/261103_1546_yolo_D_f1/trt/yolo11n_task4_D_fp32_notf32.engine
set +u; source /opt/ros/lyrical/setup.bash; set -u
export REPLAY_DOMAIN=138
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE"
WF=$(grep -m1 'world_frame:' $HOME/git/Lv2_Cartlidar_Assignment_faceyaw/workspace/ros2_ws/src/perception/config/puck_survey.yaml | awk '{print $2}')
MM=""; [ "$WF" = mission_map ] && MM="--mission-map 0.729 0.960 3.143774"
echo "survey world_frame $WF, $MM" >> $D/judge2_result.log
mkdir -p $D/f1
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 101 102 103 104 105 106; do for T in 3000 6500; do
  P=$D/f1/replay_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $SETUP -- \
    --render $B0/judge/render_${s}_T${T}_awb $MM > $P.log 2>&1 || true
  test -f $P/summary.json || echo "summary 없음: $P" >> $D/judge2_result.log
done; done
flock -u 9
echo done > $D/judge2_done.txt
