#!/usr/bin/env bash
# B0′ · P4 공식 판정 (PC). 6월드 × 색온도 2 렌더(조명 0 · 1) → AWB → replay_b0 재생(회 = 렌더 하나의 조명 하나, 모두 24회).
# 계획은 b0/plan/seed_*.json (T3000 월드 경로로 저장됨, T6500 은 --plan-world-map). 빌드 · D3 설정은 signal/run.sh 와 같음.
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
B0=$ROOT/261031_1718_r3port/b0
OUT=$B0/judge
MODEL=$ROOT/261029_1030/urdf_261028
URDF=$MODEL/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$ROOT/261031_0005/judge/signal_awbthr/build_d3/install/setup.bash
WS=$ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030_judge
export REPLAY_DOMAIN=123
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97"
for s in 101 102 103 104 105 106; do
  for T in 3000 6500; do
    R=$OUT/render_${s}_T$T
    MAP=()
    [ "$T" = 6500 ] && MAP=(--plan-world-map "$WS/seed_${s}_T3000" "$WS/seed_${s}_T6500")
    [ -f "$R/render_b0.json" ] || flock /tmp/lv2_gpu.lock timeout 1800 /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_b0.py" \
      --worlds "$WS/seed_${s}_T$T" --load-plan "$B0/plan/seed_$s.json" "${MAP[@]}" --out "$R" --/crashreporter/url= > "$R.log" 2>&1
    test -f "$R/render_b0.json"
    [ -f "$R"_awb/awb.json ] || python3 -I "$TOOLS/awb.py" --in "$R" --out "$R"_awb --temp $T > "$R.awb.log" 2>&1
  done
done
for s in 101 102 103 104 105 106; do
  for T in 3000 6500; do
    P=$OUT/replay_${s}_T$T
    [ -f "$P/summary.json" ] && continue
    REPLAY=replay_b0.py REPLAY_LAUNCH_ARGS="survey_output:=$P/survey.json" bash "$TOOLS/run_replay.sh" "$URDF" "$P" "$SETUP" -- \
      --render "$OUT/render_${s}_T$T"_awb > "$P.log" 2>&1
    test -f "$P/summary.json"
  done
done
date -Iseconds > "$OUT/done.txt"
