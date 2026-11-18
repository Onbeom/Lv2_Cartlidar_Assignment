#!/usr/bin/env bash
set -eo pipefail
cd /home/pa4/git/lv2_next_module
source /opt/ros/lyrical/setup.bash
source /home/pa4/git/ros_test/waffle-lyrical-slam/install/setup.bash
set -u
export ROS_DOMAIN_ID=92
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export CYCLONEDDS_URI=file:///home/pa4/.config/cyclonedds/cyclonedds_rx.xml
export CUVSLAM_VARIANT=V2
export ROS_LOG_DIR=/home/pa4/git/lv2_next_module/261029_1408/ros_logs
PHASE=${1:-tool}
case "$PHASE" in
  tool) RECORD=t1711_short_261027_203321 ;;
  bright) RECORD=m1711_bright_1_261027_203413 ;;
  dark) RECORD=m1711_dark_1_261027_203822 ;;
  *) echo "허용 단계: tool bright dark" >&2; exit 2 ;;
esac
flock /tmp/lv2_gpu.lock timeout --kill-after=10s 900s python3 /home/pa4/git/lv2_next_module/261029_1408/slam/run_coupled.py --record "/home/pa4/git/lv2_next_module/261027_0953/results/$RECORD" --out "/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/slam_coupled_$PHASE"
