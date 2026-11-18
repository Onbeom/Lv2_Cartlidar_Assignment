#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/new_model_261029
export ROS_DOMAIN_ID=96 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set +u
source /opt/ros/lyrical/setup.bash
set -u
python3 - "$OUT" <<'PY'
import sys,json,yaml
from pathlib import Path
out=Path(sys.argv[1]);p=json.loads((out/'perception_model_profile.json').read_text());(out/'rsp_params.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':Path(p['urdf']).read_text()}}}))
PY
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$OUT/rsp_params.yaml" > "$OUT/rsp.log" 2>&1 &
RSP=$!
cleanup(){ kill -INT "$RSP" 2>/dev/null || true; wait "$RSP" 2>/dev/null || true; }
trap cleanup EXIT
python3 "$ROOT/261029_new_model_audit/check_ros_frames.py" > "$OUT/ros_tf_check.log" 2>&1
cat "$OUT/ros_tf_check.log"
