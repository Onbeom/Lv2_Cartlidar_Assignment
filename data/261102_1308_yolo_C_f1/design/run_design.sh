#!/usr/bin/env bash
# 설계 자료: B0 설계 렌더(seed 1~6, AWB) 를 F1 과 같은 방식(현재 dev 빌드 · C 엔진)으로 재생해 관측을 모은다. PC.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261102_1308_yolo_C_f1
SETUP=${SETUP:-$D/install/setup.bash}; OUT=${OUT:-$D/design/base}
ENGINE=$ROOT/261102_1304_yolo_C/trt_results/yolo11n_task4_C_fp32_notf32.engine
TOOLS=$ROOT/Cartlidar/sim_render; B0=$ROOT/261031_1718_r3port/b0
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
export REPLAY_DOMAIN=129
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE +detector.class_names=[red_cylinder,green_cylinder,red_cuboid,green_cuboid]"
mkdir -p $OUT
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in 1 2 3 4 5 6; do
  P=$OUT/replay_$s; [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash $TOOLS/run_replay.sh $URDF $P $SETUP -- \
    --render $B0/design/render_${s}_awb --mission-map 0.729 0.960 3.143774 > $P.log 2>&1 || true
done
flock -u 9; echo done > $OUT/done.txt
