#!/bin/bash
# 열린 PR 13개 + 후방 장애물(dev 위 새 브랜치) + selfbox-1711 + barcode-cfg 합친 트리, 그리고 브랜치 단독들 빌드 · 시험 (PC)
D=/home/pa4/git/lv2_next_module/261101_0510_stale3
set +u; source /opt/ros/lyrical/setup.bash; export ROS_DOMAIN_ID=125
run() {
  mkdir -p $D/$1; cd $D/$1; touch COLCON_IGNORE
  colcon --log-base log build --base-paths $2 --packages-up-to perception perception_survey --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1
  echo "build_rc=$?" > result.log
  colcon --log-base log test --base-paths $2 --packages-select perception perception_survey --build-base build --install-base install > test.log 2>&1
  echo "test_rc=$?" >> result.log
  colcon test-result --test-result-base build --all > test_result.txt 2>&1
  tail -1 test_result.txt >> result.log
}
run build_merged $HOME/git/Lv2_Cartlidar_Assignment_merge16/workspace/ros2_ws/src
echo merged > $D/merged_done.txt
run build_rear $HOME/git/Lv2_Cartlidar_Assignment_rear2/workspace/ros2_ws/src
run build_selfbox $HOME/git/Lv2_Cartlidar_Assignment_selfbox/workspace/ros2_ws/src
run build_barcode $HOME/git/Lv2_Cartlidar_Assignment_barcodefix/workspace/ros2_ws/src
echo done > $D/build_done.txt
