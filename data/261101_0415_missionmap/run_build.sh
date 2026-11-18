#!/bin/bash
# survey mission_map: 브랜치(#24 e23f848 + 변경) · 합친 트리(인지 PR 9개 + #32 + #33 + TFmini 끊김 + 이 변경) 빌드 · 시험 (PC)
D=/home/pa4/git/lv2_next_module/261101_0415_missionmap
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
run build_merged $HOME/git/Lv2_Cartlidar_Assignment_merge13/workspace/ros2_ws/src
echo merged_done > $D/merged_done.txt
run build_branch $HOME/git/Lv2_Cartlidar_Assignment_missionmap/workspace/ros2_ws/src
echo done > $D/build_done.txt
