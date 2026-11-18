#!/usr/bin/env bash
# 노드 재생 (PC, ROS_DOMAIN_ID 97). run_node.sh <perception install setup.bash> <렌더 폴더> <결과 json>
set -u
SETUP=$1; DATA=$2; OUTJ=$3; HERE=$(dirname "$(readlink -f "$0")")
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u; source /opt/ros/lyrical/setup.bash; source "$SETUP"; set -u
python3 -c "import yaml;yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':open('$HERE/B_waffle_4wd.urdf').read()}}},open('$HERE/rsp.yaml','w'))"
set -m
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$HERE/rsp.yaml" > "$HERE/node_rsp.log" 2>&1 & A=$!
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link > /dev/null 2>&1 & B=$!
cp "$(ros2 pkg prefix perception)/share/perception/config/perception.yaml" "$HERE/perception_copy.yaml"
sed -i 's/^\(    image_qos:\) sensor_data /\1 reliable    /' "$HERE/perception_copy.yaml"
grep -q '^    image_qos: reliable' "$HERE/perception_copy.yaml" || { echo "설정 사본 고치기 실패"; exit 1; }
[ -n "${PROBE:-}" ] && { sed -i "s/^\(      axis_probe:\) [0-9.]*/\1 $PROBE/" "$HERE/perception_copy.yaml"; grep "axis_probe:" "$HERE/perception_copy.yaml"; }
ros2 run perception perception --ros-args --params-file "$HERE/perception_copy.yaml" > "$HERE/node_perception.log" 2>&1 & C=$!
trap 'kill -INT -$A -$B -$C 2>/dev/null; sleep 1; kill -KILL -$A -$B -$C 2>/dev/null' EXIT
sleep 4
python3 "$HERE/node_face_yaw.py" "$DATA" "$OUTJ" "$HOME/git/Lv2_Cartlidar_Assignment_faceyaw/workspace/ros2_ws/src/perception_survey"
