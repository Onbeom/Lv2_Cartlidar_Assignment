#!/bin/bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
ROOT=/home/pa4/git/lv2_next_module
OUT="$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325"
export ROS_DOMAIN_ID=103
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CUVSLAM_VARIANT=V2
export ROS_LOG_DIR="$ROOT/261030_1325/ros_slam_visual_logs"
exec 8>/tmp/lv2_ros_domain_103.lock
flock -w 60 8
exec 9>/tmp/lv2_gpu.lock
flock -w 3600 9
for light in bright dark; do
  export MASK_LIGHT="$light"
  export MASK_STATS="$OUT/slam_visual_${light}/mask_stats.json"
  if [ "$light" = bright ]; then record=m1711_bright_1_261027_203413; else record=m1711_dark_1_261027_203822; fi
  timeout --signal=INT --kill-after=30s 900 python3 "$ROOT/261030_1325/slam_visual/run_coupled.py" --record "$ROOT/261027_0953/results/$record" --out "$OUT/slam_visual_$light" > "$OUT/slam_visual_${light}.log" 2>&1
  /home/pa4/git/ros_test/waffle-lyrical-slam/install/cartographer_ros/lib/cartographer_ros/cartographer_pbstream_to_ros_map -pbstream_filename "$OUT/slam_visual_$light/map.pbstream" -map_filestem "$OUT/slam_visual_$light/map" -resolution 0.025 > "$OUT/slam_visual_${light}_to_map.log" 2>&1
done
