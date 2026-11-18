#!/bin/bash
# 다시: 옛 기준 브랜치 2개는 perception 만(그때는 perception_survey 가 없었음), 후방 장애물 브랜치는 시험만 다시(첫 번째는 CPU 부하 중 SignalLight 무작위 시험 60 s 제한 넘김)
D=/home/pa4/git/lv2_next_module/261101_0510_stale3
set +u; source /opt/ros/lyrical/setup.bash; export ROS_DOMAIN_ID=125
run() {
  rm -rf $D/$1; mkdir -p $D/$1; cd $D/$1; touch COLCON_IGNORE
  colcon --log-base log build --base-paths $2 --packages-up-to perception --build-base build --install-base install --event-handlers console_direct- > build.log 2>&1
  echo "build_rc=$?" > result.log
  colcon --log-base log test --base-paths $2 --packages-select perception --build-base build --install-base install > test.log 2>&1
  echo "test_rc=$?" >> result.log
  colcon test-result --test-result-base build --all > test_result.txt 2>&1
  tail -1 test_result.txt >> result.log
}
until grep -q done $D/checks.log 2>/dev/null; do sleep 10; done
cd $D/build_rear && colcon --log-base log test --base-paths $HOME/git/Lv2_Cartlidar_Assignment_rear2/workspace/ros2_ws/src --packages-select perception perception_survey --build-base build --install-base install > test2.log 2>&1
echo "test2_rc=$?" > result2.log; colcon test-result --test-result-base build --all > test_result2.txt 2>&1; tail -1 test_result2.txt >> result2.log
run build_selfbox $HOME/git/Lv2_Cartlidar_Assignment_selfbox/workspace/ros2_ws/src
run build_barcode $HOME/git/Lv2_Cartlidar_Assignment_barcodefix/workspace/ros2_ws/src
echo done > $D/build2_done.txt
