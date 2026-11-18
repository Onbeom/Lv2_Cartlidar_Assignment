#!/usr/bin/env bash
# loc_cpp 비교 재생 (PC). 따로 도메인 · LOCALHOST. 결과 out/ (비교 bag, launch 로그)
set -u
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
B=/tmp/claude-1000/-home-pa4-git-lv2-next-module/f89e5d4b-7cc0-4ca9-b419-c0309f318e34/scratchpad/loccpu_build/install
rm -rf "$D/out"; mkdir -p "$D/out"
E="env -i HOME=$HOME USER=$USER PATH=/usr/local/bin:/usr/bin:/bin LANG=en_US.UTF-8 ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
S="source /opt/ros/lyrical/setup.bash && source $B/setup.bash"
$E bash -c "$S && exec ros2 launch $D/dual.launch.py" > "$D/out/launch.log" 2>&1 &
L=$!
sleep 12
$E bash -c "$S && exec ros2 bag record -o $D/out/cmp --disable-keyboard-controls --use-sim-time --qos-profile-overrides-path $D/qos.yaml --topics /localization/status /cpp/localization/status /localization/lock /cpp/localization/lock /initialpose /cpp/initialpose /scan_merged /cpp/scan_merged /scan_front_filtered /cpp/scan_front_filtered /scan_rear_filtered /cpp/scan_rear_filtered /amcl_pose" < /dev/null > "$D/out/record.log" 2>&1 &
R=$!
sleep 4
$E bash -c "$S && ros2 bag play $D/bag_replay --clock 100 --disable-keyboard-controls --qos-profile-overrides-path $D/play_qos.yaml" < /dev/null > "$D/out/play.log" 2>&1
sleep 5
# 11-16: 백그라운드 작업은 SIGINT 를 무시함 → TERM 으로 (rosbag2 · launch 둘 다 TERM 에도 정리함)
kill -TERM $R; sleep 4; kill -TERM $L; sleep 8; pkill -TERM -f "loccpu_build/install|nav2_map_server/map_server|nav2_amcl/amcl" 2>/dev/null
pgrep -af "loccpu_build/install|lib/localization/|nav2_amcl/amcl|nav2_map_server" | grep -v pgrep | cut -c1-100 || true
echo "끝: $(date +%T)"; tail -2 "$D/out/play.log"
