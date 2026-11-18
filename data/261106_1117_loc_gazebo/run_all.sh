#!/bin/bash
# T1a → T1b → T2b → T2a. 끝나면 results/done.txt
H=/home/pa4/git/lv2_next_module/261106_1117_loc_gazebo
M=$H/ws/install/mission_planner/share/mission_planner/maps/map_amcl.yaml
bash $H/run_one.sh T1a_stop_match stop $M
bash $H/run_one.sh T1b_stop_default stop ""
bash $H/run_one.sh T2b_drive_match drive $M
bash $H/run_one.sh T2a_drive_default drive ""
date > $H/results/done.txt
