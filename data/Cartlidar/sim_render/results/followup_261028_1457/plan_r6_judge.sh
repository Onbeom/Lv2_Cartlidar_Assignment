#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
/home/pa4/isaacsim-venv/bin/python Cartlidar/sim_render/render_r6_judge.py --worlds Cartlidar/sim_world/worlds/dice_mission/seed_{1,2,3,4,5,6} --drawing Cartlidar/sim_world/data/maze_dice_mission.json --out Cartlidar/sim_render/results/followup_261028_1457/r6_judge_render --seed 20261102 --height 0.20 --only-plan --view-cache Cartlidar/sim_render/results/followup_261028_1457/r6_design3_plan.json --plan-file Cartlidar/sim_render/results/followup_261028_1457/r6_judge_plan.json > Cartlidar/sim_render/results/followup_261028_1457/r6_judge_plan.log 2>&1
