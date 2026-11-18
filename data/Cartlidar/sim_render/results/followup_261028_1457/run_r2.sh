#!/usr/bin/env bash
set -euo pipefail
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261028_1457/ros_logs
export REPLAY=replay_r12.py
bash /home/pa4/git/lv2_next_module/Cartlidar/sim_render/run_replay.sh /home/pa4/git/lv2_next_module/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf /home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/replay_r2 /home/pa4/git/lv2_next_module/261028_1457/install/setup.bash -- --render /home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/mission_261028/render_r12_gpu r2 --mission --runs-per-group 20 --rate 1 > /home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/replay_r2.log 2>&1
