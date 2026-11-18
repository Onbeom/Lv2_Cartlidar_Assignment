#!/bin/bash
HERE=/home/pa4/git/lv2_next_module/261031_1325_nav2
OUT=$HERE/results/DIAG_costmap; mkdir -p $OUT
URDF=/home/pa4/git/lv2_next_module/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=102 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/test/ros_logs
exec 8>/tmp/lv2_ros_domain_102.lock; flock -w 60 8 || exit 1
setsid /usr/bin/python3 $HERE/nodes/sim2d.py --ros-args -p log:=$OUT/truth.csv > $OUT/sim.log 2>&1 & SP=$!
sleep 1
setsid ros2 launch $HERE/launch/nav2_mission.launch.py use_sim_time:=true use_rsp:=true urdf:=$URDF > $OUT/launch.log 2>&1 & LP=$!
sleep 20
/usr/bin/python3 $HERE/test/dump_costmap.py $OUT/costmaps.npz
timeout 10 ros2 topic echo --once /amcl_pose > $OUT/amcl_pose.txt 2>&1
for p in $LP $SP; do kill -INT -$p; done; sleep 6; for p in $LP $SP; do kill -KILL -$p 2>/dev/null; done
