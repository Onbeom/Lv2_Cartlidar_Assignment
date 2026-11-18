#!/usr/bin/env bash
# 새 미로 R1′ · R2′ 판정 (PC). 색온도 월드마다: 렌더 → AWB → R1′ 재생 → R2′ 재생. 기준 README.md
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
TOOLS=$ROOT/Cartlidar/sim_render
OUT=$ROOT/261031_0005/judge/signal
MODEL=$ROOT/261029_1030/urdf_261028
URDF=$MODEL/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
USD=$MODEL/isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda
SETUP=$ROOT/261031_0005/judge_build/install/setup.bash
export REPLAY_DOMAIN=121
mkdir -p "$OUT"
for T in 3000 6500; do
  W=$ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030_judge/seed_101_T$T
  R=$OUT/render_T$T
  if [ ! -f "$R/render_r12.json" ]; then
    rm -rf "$R"
    flock /tmp/lv2_gpu.lock timeout 1800 /home/pa4/isaacsim-venv/bin/python "$TOOLS/render_r12.py" --worlds "$W" \
      --urdf "$URDF" --robot-usd "$USD" --arm-params "$MODEL/isaac/robot_params.json" \
      --drawing "$ROOT/Cartlidar/sim_world/data/maze_dice_mission_maze261030.json" \
      --start-pose 0.734 0.950 180 --poses 20 --out "$R" --/crashreporter/url= > "$OUT/render_T$T.log" 2>&1
  fi
  [ -d "$R"_awb ] || python3 -I "$TOOLS/awb.py" --in "$R" --out "$R"_awb --temp $T > "$OUT/awb_T$T.log" 2>&1
  REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r1_T$T" "$SETUP" -- --render "$R"_awb r1 > "$OUT/replay_r1_T$T.log" 2>&1
  test -f "$OUT/replay_r1_T$T/summary.json"
  REPLAY=replay_r12.py bash "$TOOLS/run_replay.sh" "$URDF" "$OUT/replay_r2_T$T" "$SETUP" -- --render "$R"_awb r2 --mission --all-counts --runs-per-group 25 --rate 1 > "$OUT/replay_r2_T$T.log" 2>&1
  test -f "$OUT/replay_r2_T$T/summary.json"
done
date -Iseconds > "$OUT/done.txt"
