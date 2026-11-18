#!/bin/bash
H=/home/pa4/git/lv2_next_module/261106_1152_gap_fix
M=/home/pa4/git/lv2_next_module/261106_1117_loc_gazebo/ws/install/mission_planner/share/mission_planner/maps/map_amcl.yaml
bash $H/watchdog.sh & W=$!
KILL_AMCL_AFTER=25 bash $H/run_gz.sh F3_amcl_kill drive $M
bash $H/run_gz.sh F1_drive_match drive $M
LOCMON=false bash $H/run_gz.sh F0_no_monitor drive $M
EXTRA_ARGS="loc_res_thr:=0.10" bash $H/run_gz.sh F4_thr010 drive $M
bash $H/run_gz.sh F2_drive_match drive $M
bash $H/run_isaac.sh
date > $H/results/done.txt
wait $W
