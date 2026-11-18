#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261028_1457/ros_logs_r4_regression
export REPLAY=replay_r4.py
bash Cartlidar/sim_render/run_replay_r4_regression.sh 261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf Cartlidar/sim_render/results/followup_261028_1457/r4_regression /home/pa4/git/lv2_next_module/261028_1457/install/setup.bash -- --render Cartlidar/sim_render/results/render_r4_1711_aim_261027 arm > Cartlidar/sim_render/results/followup_261028_1457/r4_regression.log 2>&1
test -s Cartlidar/sim_render/results/followup_261028_1457/r4_regression/summary.json
