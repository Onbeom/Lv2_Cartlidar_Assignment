#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
PHASE=${1:-r6_design}
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261028_1457/ros_logs_r6
export REPLAY=replay_r6_multiview.py
bash Cartlidar/sim_render/run_replay_r6_multi.sh 261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_replay /home/pa4/git/lv2_next_module/261028_1457/install/setup.bash -- --render Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_render > Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_replay.log 2>&1
test -s Cartlidar/sim_render/results/followup_261028_1457/${PHASE}_replay/summary.json
