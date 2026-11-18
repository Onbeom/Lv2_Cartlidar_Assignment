#!/usr/bin/env bash
# F1b (집게 상자 거르기 뒤) 최종 인지 연결 판정 (PC, 모의): B0′ 판정 렌더 · REPLAY_SET 은 판정 2차와 같고, 빌드는 합친 트리 + #32,
# 주 검출기는 YOLO 후보 B 엔진(PC FP32). GPU 잠금 안에서 차례로 12번 재생(조명 0 · 1 = 회 2, 모두 24회).
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
B0=$ROOT/261031_1718_r3port/b0
OUT=$ROOT/261101_0246_final/f1b
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$ROOT/261101_0246_final/sb/build_merged/install/setup.bash
ENGINE=$ROOT/261031_0100_trt/results/yolo11n_task4_B_fp32_notf32.engine
export REPLAY_DOMAIN=${REPLAY_DOMAIN:-124}
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97 detector.engine_path=$ENGINE +detector.class_names=[red_cylinder,green_cylinder,red_cuboid,green_cuboid]"
sha256sum "$0" "$ENGINE" "$TOOLS/set_yaml_values.py" "$TOOLS/run_replay.sh" "$TOOLS/replay_b0.py" > "$OUT/source_sha256.txt"
exec 9>/tmp/lv2_gpu.lock; flock 9
for s in ${SEEDS:-101 102 103 104 105 106}; do
  for T in 3000 6500; do
    P=$OUT/replay_${s}_T$T
    [ -f "$P/summary.json" ] && continue
    REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash "$TOOLS/run_replay.sh" "$URDF" "$P" "$SETUP" -- \
      --render "$B0/judge/render_${s}_T$T"_awb > "$P.log" 2>&1 || true
    test -f "$P/summary.json" || echo "summary 없음: $P"
  done
done
flock -u 9
date > "$OUT/done.txt"
