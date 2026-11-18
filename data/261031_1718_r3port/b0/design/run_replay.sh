#!/usr/bin/env bash
# 설계 자료 재생: run_replay.sh <이름> <setup.bash>  — seed 1~6 렌더(AWB)를 그 빌드로 재생 → replay_<이름>_<s>/
set -euo pipefail
NAME=$1; SETUP=$2
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$ROOT/261031_1718_r3port/b0/design
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
export REPLAY_DOMAIN=${REPLAY_DOMAIN:-124}
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97"
for s in 1 2 3 4 5 6; do
  P=$OUT/replay_${NAME}_$s
  [ -f "$P/summary.json" ] && continue
  REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash "$TOOLS/run_replay.sh" "$URDF" "$P" "$SETUP" -- \
    --render "$OUT/render_${s}_awb" > "$P.log" 2>&1
  test -f "$P/summary.json"
done
date -Iseconds > "$OUT/replay_${NAME}_done.txt"
