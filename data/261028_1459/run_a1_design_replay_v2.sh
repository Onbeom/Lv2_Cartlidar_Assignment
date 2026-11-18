#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
OUT=/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/a1_code_261028
REPLAY=replay_a1.py bash 261028_1459/run_replay_a1_private.sh /home/pa4/git/lv2_next_module/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf "$OUT/replay_design_v2" /home/pa4/git/lv2_next_module/261028_1459/a1_work/a1_fix_build/install/setup.bash -- --render /home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/render_a1_1711_design_261027 > "$OUT/replay_design_v2.log" 2>&1
test -f "$OUT/replay_design_v2/summary.json"
