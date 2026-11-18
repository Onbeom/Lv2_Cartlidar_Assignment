#!/usr/bin/env bash
# R1′ · R2′ 재판정 — 팔 · 뿌리 자리 고친 render_r12 (PC). 기준 · 인지 빌드 · D3 설정은 261031_0005/judge/signal_awbthr/run.sh 와 같음.
# 바뀐 것: 렌더 도구만(새 모델 팔을 USD 부모 링크 기준으로, 뿌리 base_link 6.75 mm). 3000 · 6500 K 모두 다시 렌더.
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
J=$ROOT/261031_0005/judge
OUT=$ROOT/261031_1718_r3port/signal
MODEL=$ROOT/261029_1030/urdf_261028
URDF=$MODEL/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
USD=$MODEL/isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda
SETUP=$J/signal_awbthr/build_d3/install/setup.bash
export REPLAY_DOMAIN=122
export REPLAY_SET="signal.s_min=75 signal.red_v_min=180 signal.red_warm_h_min=140 signal.red_warm_v_min=150 signal.yellow_v_min=218 signal.yellow_h_max=67 signal.green_h_min=68 signal.green_h_max=97"
for T in 3000 6500; do
  W=$ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030_judge/seed_101_T$T
  R=$OUT/render_T$T
  [ -f "$R/render_r12.json" ] || flock /tmp/lv2_gpu.lock timeout 1800 /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_r12.py" --worlds "$W" \
    --urdf "$URDF" --robot-usd "$USD" --arm-params "$MODEL/isaac/robot_params.json" \
    --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission_maze261030.json" \
    --start-pose 0.734 0.950 180 --poses 20 --out "$R" --/crashreporter/url= > "$R.log" 2>&1
  python3 -I "$J/vmin_design/ghost_check.py" "$R" > "$R.ghost.txt" || { cat "$R.ghost.txt"; exit 1; }
  [ -d "$R"_awb ] || python3 -I "$TOOLS/awb.py" --in "$R" --out "$R"_awb --temp $T > "$OUT/awb_T$T.log" 2>&1
done
for T in 3000 6500; do
  REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r1_T$T" "$SETUP" -- --render "$OUT/render_T$T"_awb r1 > "$OUT/replay_r1_T$T.log" 2>&1
  test -f "$OUT/replay_r1_T$T/summary.json"
  REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r2_T$T" "$SETUP" -- --render "$OUT/render_T$T"_awb r2 --mission --all-counts --runs-per-group 25 --rate 1 > "$OUT/replay_r2_T$T.log" 2>&1
  test -f "$OUT/replay_r2_T$T/summary.json"
done
date -Iseconds > "$OUT/done.txt"
