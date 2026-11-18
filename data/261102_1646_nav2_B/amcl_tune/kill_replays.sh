#!/bin/bash
# 재생 도구 프로세스 모두 끝냄(Isaac 라이브 실행 · YOLO 는 안 건드림: 도메인 110~ 재생 노드만)
pkill -f "replay_amcl.sh"
pkill -INT -f "ros2 bag play .*/out_bag --rate"
sleep 3
pkill -f "amcl_tune/odom_to_tf.py"; pkill -f "amcl_tune/loc_eval.py"
pkill -f "params-file .*amcl_tune/params"
pkill -f "__node:=lifecycle_manager_localization -p use_sim_time:=true -p autostart:=true -p node_names:=\[map_server,amcl\]"
for p in $(pgrep -f "scan_body_mask.py --ros-args -p use_sim_time:=true$"); do kill $p; done
for p in $(pgrep -f "scan_merge.py --ros-args -p use_sim_time:=true$"); do kill $p; done
sleep 3
pgrep -af "amcl_tune|params-file .*amcl_tune" | grep -v kill_replays
