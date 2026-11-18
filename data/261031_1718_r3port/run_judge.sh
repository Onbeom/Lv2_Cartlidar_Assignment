#!/usr/bin/env bash
# R3′ 판정 (PC): 계획 합치기 → 색온도마다 렌더(GPU 잠금) → AWB → C++ 판독 → 집계. 기준 README.md
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
D=$ROOT/261031_1718_r3port
M=$ROOT/261029_1030/urdf_261028
W=$ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030_judge
BC=$ROOT/261031_0005/judge/signal_awbthr/build_d3/install/perception/lib/perception/barcode_compare
python3 $D/merge_plan.py
for T in 3000 6500; do
  R=$D/render_T$T
  if [ ! -f "$R/render_r3.json" ]; then
    rm -rf "$R"
    MAP=()
    WORLDS=()
    for s in 101 102 103 104 105 106; do
      WORLDS+=("$W/seed_${s}_T$T")
      [ $T = 3000 ] || MAP+=(--plan-world-map "$W/seed_${s}_T3000" "$W/seed_${s}_T$T")
    done
    flock /tmp/lv2_gpu.lock timeout 3600 ~/isaacsim-venv/bin/python $ROOT/Cartlidar/sim_render/render_r3.py --mission \
      --worlds "${WORLDS[@]}" --urdf $M/isaac/urdf/waffle_4wd_light.urdf --arm-frame arm_camera_optical_frame \
      --robot-usd $M/isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda --arm-params $M/isaac/robot_params.json \
      --drawing $ROOT/Cartlidar/sim_world/data/maze_dice_mission_maze261030.json \
      --load-plan $D/plan/r3_plan_T3000.json "${MAP[@]}" --out "$R" --/crashreporter/url= > $D/render_T$T.log 2>&1
  fi
  [ -d "$R"_awb ] || python3 -I $ROOT/Cartlidar/sim_render/awb.py --in "$R" --out "$R"_awb --temp $T > $D/awb_T$T.log 2>&1
  python3 -c "
import json,sys
r=json.load(open('$R'+'_awb/render_r3.json'))
print('\n'.join(p['name']+' '+'$R'+'_awb/'+p['images']['color'] for p in r['poses']))" > $D/read_list_T$T.txt
  $BC --read $D/read_list_T$T.txt > $D/read_T$T.jsonl 2> $D/read_T$T.err
done
python3 $D/judge_r3.py
date -Iseconds > $D/done.txt
