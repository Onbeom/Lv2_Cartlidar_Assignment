#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
PHASE=${1:-r6_design}
if [ "$PHASE" = "r6_fresh_all" ]; then
  for seed in 1 2 3 4 5 6; do
    target="Cartlidar/sim_render/results/followup_261028_1457/r6_fresh_seed${seed}_render"
    if [ -s "$target/render_r6.json" ]; then continue; fi
    echo "독립 렌더 seed$seed"
    flock /tmp/lv2_gpu.lock timeout --kill-after=10s 600s /home/pa4/isaacsim-venv/bin/python Cartlidar/sim_render/render_r6_judge.py --worlds "Cartlidar/sim_world/worlds/dice_mission/seed_${seed}" --drawing Cartlidar/sim_world/data/maze_dice_mission.json --out "$target" --seed 20261103 --height 0.20 --lying-prob 0.5 --load-plan "Cartlidar/sim_render/results/followup_261028_1457/r6_fresh_seed${seed}_plan.json" --/crashreporter/url= > "Cartlidar/sim_render/results/followup_261028_1457/r6_fresh_seed${seed}_render.log" 2>&1
    test -s "$target/render_r6.json"
  done
  python3 Cartlidar/sim_render/results/followup_261028_1457/merge_r6_fresh.py
  exit 0
fi
if [ "$PHASE" = "r6_holdout_all" ]; then
  for seed in 1 2 3 4 5 6; do
    target="Cartlidar/sim_render/results/followup_261028_1457/r6_holdout_seed${seed}_render"
    if [ -s "$target/render_r6.json" ]; then continue; fi
    echo "독립 렌더 seed$seed"
    flock /tmp/lv2_gpu.lock timeout --kill-after=10s 600s /home/pa4/isaacsim-venv/bin/python Cartlidar/sim_render/render_r6_judge.py --worlds "Cartlidar/sim_world/worlds/dice_mission/seed_${seed}" --drawing Cartlidar/sim_world/data/maze_dice_mission.json --out "$target" --seed 20261104 --height 0.20 --lying-prob 0.5 --load-plan "Cartlidar/sim_render/results/followup_261028_1457/r6_holdout_seed${seed}_plan.json" --/crashreporter/url= > "Cartlidar/sim_render/results/followup_261028_1457/r6_holdout_seed${seed}_render.log" 2>&1
    test -s "$target/render_r6.json"
  done
  python3 Cartlidar/sim_render/results/followup_261028_1457/merge_r6_holdout.py
  exit 0
fi
flock /tmp/lv2_gpu.lock timeout --kill-after=10s 180s /home/pa4/isaacsim-venv/bin/python Cartlidar/sim_render/render_r6_multiview.py --worlds Cartlidar/sim_world/worlds/dice_mission/seed_1 --drawing Cartlidar/sim_world/data/maze_dice_mission.json --out Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_render --seed 20261101 --case-limit 1 --height 0.20 --load-plan Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_plan.json --/crashreporter/url= > Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_render.log 2>&1
test -s Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_render/render_r6.json
