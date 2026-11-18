#!/usr/bin/env bash
# B0′ · P4 판정 2차: 판정 1차와 같은 렌더(../judge/render_*_awb) · 설정, 인지 빌드만 설계 1(design/build_s1).
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
B0=$ROOT/261031_1718_r3port/b0
OUT=$B0/judge2
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$B0/design/build_s1/install/setup.bash
export REPLAY_DOMAIN=${REPLAY_DOMAIN:-123}
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97"
for s in $SEEDS; do
  for T in 3000 6500; do
    P=$OUT/replay_${s}_T$T
    [ -f "$P/summary.json" ] && continue
    REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash "$TOOLS/run_replay.sh" "$URDF" "$P" "$SETUP" -- \
      --render "$B0/judge/render_${s}_T$T"_awb > "$P.log" 2>&1
    test -f "$P/summary.json"
  done
done
