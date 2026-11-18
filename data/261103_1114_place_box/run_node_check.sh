#!/usr/bin/env bash
# 노드 확인 실행 (PC, ROS_DOMAIN_ID 93). run_node_check.sh <perception install setup.bash> <렌더 폴더> <장면 수>
set -u
SETUP=$1; DATA=$2; NS=$3; HERE=$(dirname "$(readlink -f "$0")")
export ROS_DOMAIN_ID=93 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u; source /opt/ros/lyrical/setup.bash; source "$SETUP"; set -u
python3 -c "import yaml,sys;yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':open('$HERE/B_waffle_4wd.urdf').read()}}},open('$HERE/rsp.yaml','w'))"
set -m
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$HERE/rsp.yaml" > "$HERE/node_rsp.log" 2>&1 & A=$!
ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link > /dev/null 2>&1 & B=$!
# 설정 사본: 영상 QoS reliable (재생기가 reliable 로 내고, 큰 영상이 best effort 에서 버려지지 않게 — A1 재생 18차와 같음)
cp "$(ros2 pkg prefix perception)/share/perception/config/perception.yaml" "$HERE/perception_copy.yaml"
sed -i 's/^\(    image_qos:\) sensor_data /\1 reliable    /' "$HERE/perception_copy.yaml"
grep -q '^    image_qos: reliable' "$HERE/perception_copy.yaml" || { echo "설정 사본 고치기 실패"; exit 1; }
ros2 run perception perception --ros-args --params-file "$HERE/perception_copy.yaml" > "$HERE/node_perception.log" 2>&1 & C=$!
trap 'kill -INT -$A -$B -$C 2>/dev/null; sleep 1; kill -KILL -$A -$B -$C 2>/dev/null' EXIT
sleep 4
python3 "$HERE/node_check.py" "$DATA" "$NS"
