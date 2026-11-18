#!/bin/bash
# 브랜치 단독 · 합친 트리 + 이 변경 빌드 · 시험 (PC)
D=/home/pa4/git/lv2_next_module/261101_0040_yolopost
set +u; source /opt/ros/lyrical/setup.bash; export ROS_DOMAIN_ID=124
run() {  # $1 이름  $2 src
  mkdir -p $D/$1; cd $D/$1; touch COLCON_IGNORE
  colcon --log-base log build --base-paths $2 --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1
  echo "build_rc=$?" > result.log
  colcon --log-base log test --base-paths $2 --packages-select perception perception_survey --build-base build --install-base install > test.log 2>&1
  echo "test_rc=$?" >> result.log
  colcon test-result --test-result-base build --all > test_result.txt 2>&1
  tail -1 test_result.txt >> result.log
}
touch $D/COLCON_IGNORE
run build_branch $HOME/git/Lv2_Cartlidar_Assignment_yolopost/workspace/ros2_ws/src
run build_merged $HOME/git/Lv2_Cartlidar_Assignment_merge9yp/workspace/ros2_ws/src
echo done > $D/build_done.txt
