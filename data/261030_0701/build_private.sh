#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
set +u
source /opt/ros/lyrical/setup.bash
set -u
cd "$ROOT/261030_0701"
colcon build --base-paths src --packages-select robot_interfaces perception perception_survey --cmake-args -DCMAKE_BUILD_TYPE=Release > "$ROOT/Cartlidar/sim_render/results/integration_261030_0701/build.log" 2>&1
./build/perception/test_perception_logic > "$ROOT/Cartlidar/sim_render/results/integration_261030_0701/gtest.txt" 2>&1
python3 -m pytest -q src/perception_survey/test > "$ROOT/Cartlidar/sim_render/results/integration_261030_0701/b0_pytest.txt" 2>&1
