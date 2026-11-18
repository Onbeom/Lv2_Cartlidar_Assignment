#!/usr/bin/env bash
# 렌더 영상 ROS 재생 시험 실행 (PC, Isaac Sim 없이). robot_state_publisher(URDF) + 인지 노드를 띄우고 replay_ros.py 를 부른다.
#   run_replay.sh <URDF> <결과 폴더> <perception install setup.bash> -- <replay_ros.py 인자...>
# 설정은 설치된 perception/config 를 결과 폴더 config/ 로 복사해 고친 사본을 쓴다(원본 설정은 그대로):
#   image_qos: reliable — 재생기가 reliable 로 내고, 큰 영상이 best effort 에서 버려지지 않게(18차 기록)
# 다른 세션 토픽과 섞이지 않게 ROS_DOMAIN_ID=87, 같은 PC 안 통신만. 끝나면 띄운 프로세스를 모두 끈다.
set -u
URDF=${1:?사용: run_replay.sh <URDF> <결과 폴더> <setup.bash> -- <replay 인자>}
OUT=${2:?결과 폴더}
SETUP=${3:?perception install setup.bash}
shift 3
[ "${1:-}" = "--" ] && shift
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
[ -n "$(ls -A "$OUT")" ] && { echo "빈 폴더가 아님: $OUT (다른 폴더 사용)"; exit 1; }

export ROS_DOMAIN_ID=88 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
set +u
source /opt/ros/lyrical/setup.bash
source "$SETUP"
set -u

SHARE=$(ros2 pkg prefix perception)/share/perception/config
cp -r "$SHARE" "$OUT/config"
sed -i 's/^\(    image_qos:\) sensor_data /\1 reliable    /' "$OUT/config/perception.yaml"
grep -q '^    image_qos: reliable' "$OUT/config/perception.yaml" || { echo "설정 사본 고치기 실패"; exit 1; }
python3 - "$OUT/config/perception.yaml" <<'CFG'
import sys,yaml
from pathlib import Path
p=Path(sys.argv[1]);data=yaml.safe_load(p.read_text());params=data['perception_node']['ros__parameters'];params.setdefault('tag',{}).update({'prefer_horizontal_face':True,'detect_gamma':2.0,'upscale_all':True,'horizontal_heights_m':[0.1302,0.1002],'support_top_z_m':0.07,'support_tag_offset_m':0.0002});p.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
CFG
export DICE_OBSERVATION_LOG="$OUT/dice_observations.jsonl"
diff "$SHARE/perception.yaml" "$OUT/config/perception.yaml" > "$OUT/config_diff.txt"

python3 - "$URDF" "$OUT/rsp_params.yaml" <<'EOF'
import sys, yaml
yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
    'robot_description': open(sys.argv[1]).read(), 'use_sim_time': True}}}, open(sys.argv[2], 'w'))
EOF

set -m
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$OUT/rsp_params.yaml" > "$OUT/rsp.log" 2>&1 &
RSP=$!
ros2 launch perception perception.launch.py use_sim_time:=true config_dir:="$OUT/config" > "$OUT/perception_launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/dice_multiview.py" --ros-args -p use_sim_time:=true > "$OUT/dice_multiview.log" 2>&1 &
MULTI=$!
cleanup() {
  kill -INT -"$MULTI" 2>/dev/null
  kill -INT -"$LAUNCH" -"$RSP" 2>/dev/null
  for _ in $(seq 1 50); do
    kill -0 -"$LAUNCH" 2>/dev/null || kill -0 -"$RSP" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -"$LAUNCH" -"$RSP" -"$MULTI" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM
sleep 4
python3 "$HERE/${REPLAY:-replay_ros.py}" --out "$OUT" "$@" 2>&1 | tee "$OUT/replay.out"   # REPLAY=replay_r12.py 로 묶음 재생
