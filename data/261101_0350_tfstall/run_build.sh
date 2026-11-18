#!/bin/bash
# TFmini 끊김 수정 브랜치(dev + 변경) 빌드 · 시험 (PC)
D=/home/pa4/git/lv2_next_module/261101_0350_tfstall
set +u; source /opt/ros/lyrical/setup.bash; export ROS_DOMAIN_ID=125
mkdir -p $D/build_branch; cd $D/build_branch; touch COLCON_IGNORE
S=$HOME/git/Lv2_Cartlidar_Assignment_tfstall/workspace/ros2_ws/src
colcon --log-base log build --base-paths $S --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1
echo "build_rc=$?" > result.log
colcon --log-base log test --base-paths $S --packages-select perception perception_survey --build-base build --install-base install > test.log 2>&1
echo "test_rc=$?" >> result.log
colcon test-result --test-result-base build --all > test_result.txt 2>&1
tail -1 test_result.txt >> result.log
echo done > $D/build_done.txt
