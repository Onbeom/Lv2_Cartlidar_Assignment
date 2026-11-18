#!/usr/bin/env bash
# R3′ 계획(새 모델, CPU 만): 판정 월드 seed 101~106 (T3000, 배치는 T6500 과 같음) 를 월드마다 따로 계획 → plan/seed_<n>.json
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module
D=$ROOT/261031_1718_r3port
M=$ROOT/261029_1030/urdf_261028
for s in 101 102 103 104 105 106; do
  ~/isaacsim-venv/bin/python $ROOT/Cartlidar/sim_render/render_r3.py --mission \
    --worlds $ROOT/Cartlidar/sim_world/worlds/dice_mission_maze261030_judge/seed_${s}_T3000 \
    --urdf $M/isaac/urdf/waffle_4wd_light.urdf --arm-frame arm_camera_optical_frame \
    --drawing $ROOT/Cartlidar/sim_world/data/maze_dice_mission_maze261030.json \
    --seed 20261031$s --search-bases 12 --out $D/plan/unused_$s --only-plan --save-plan $D/plan/seed_$s.json \
    > $D/plan/seed_$s.log 2>&1 &
done
wait
date -Iseconds > $D/plan/done.txt
