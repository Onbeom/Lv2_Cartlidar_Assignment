#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/pa4/git/lv2_next_module
source /opt/ros/lyrical/setup.bash
colcon --log-base "$ROOT/261030_0701/team_verify_a1/log" build --base-paths /home/pa4/git/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src --build-base "$ROOT/261030_0701/team_verify_a1/build" --install-base "$ROOT/261030_0701/team_verify_a1/install" --packages-up-to perception --cmake-args -DCMAKE_BUILD_TYPE=Release > "$ROOT/Cartlidar/sim_render/results/integration_261030_0701/a1_team_build.log" 2>&1
"$ROOT/261030_0701/team_verify_a1/build/perception/test_perception_logic" > "$ROOT/Cartlidar/sim_render/results/integration_261030_0701/a1_team_gtest.txt" 2>&1
