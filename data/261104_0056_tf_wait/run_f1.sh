#!/usr/bin/env bash
# 회귀: 퍽 측량 F1 24회 (D 엔진, 이 브랜치 빌드). 재생기는 TF 를 영상보다 먼저 내므로 늘 '가장 새 장' 길만 탄다
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261104_0056_tf_wait
SETUP=$D/build/install/setup.bash
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
ENGINE=$ROOT/261103_1546_yolo_D_f1/trt/yolo11n_task4_D_fp32_notf32.engine
{ sha256sum $0; git -C $HOME/git/Lv2_Cartlidar_Assignment_tfwait rev-parse HEAD; git -C $HOME/git/Lv2_Cartlidar_Assignment_tfwait diff | sha256sum; } > $D/source_sha256_f1.txt
set +u; source /opt/ros/lyrical/setup.bash; set -u
export REPLAY_DOMAIN=138
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE"
mkdir -p $D/f1
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 101 102 103 104 105 106; do for T in 3000 6500; do
  P=$D/f1/replay_${s}_T$T; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $SETUP -- \
    --render $B0/judge/render_${s}_T${T}_awb > $P.log 2>&1 || true
  test -f $P/summary.json || echo "summary 없음: $P" >> $D/f1_result.log
done; done
flock -u 9
echo done > $D/f1_done.txt
