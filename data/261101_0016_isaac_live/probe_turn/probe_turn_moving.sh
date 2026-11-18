#!/bin/bash
# 도구 확인(모의, 판정 아님): 빈 바닥(empty_maze.usda)에 scene_live.py(cmd_vel 모드)를 띄우고 probe_turn_moving.py 로 전진 중 회전 곡선을 잰다.
# PC, ROS 도메인 102 · GPU 잠금(run_live_nav2.sh 와 같음). 사용: probe_turn_moving.sh <이름> [probe 추가 인자]
set -o pipefail
NAME=$1; DOM=102; EXTRA=${2:-}
TOP=/home/pa4/git/lv2_next_module
HERE=$TOP/261101_0016_isaac_live
OUT=$HERE/results/$NAME
LAB_PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
PYLINKS=$TOP/261025_0307/bridge_check/pylinks
[ -e $OUT ] && { echo "$OUT 이미 있음"; exit 1; }
mkdir -p $OUT
exec 8>/tmp/lv2_ros_domain_$DOM.lock; flock -w 60 8 || { echo "도메인 $DOM 잠금 실패"; exit 1; }
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || { echo "GPU 잠금 실패"; exit 1; }
set +u
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=$DOM ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOG_DIR=$HERE/ros_logs
sha256sum $HERE/scene_live.py $HERE/probe_turn/* > $OUT/source_sha256.txt
STOP=$OUT/stop_scene
env -u AMENT_PREFIX_PATH bash -c "export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$LAB_PY' '$HERE/scene_live.py' --out '$OUT/scene' --light_level 1.0 --mode cmd_vel \
    --maze '$HERE/probe_turn/empty_maze.usda' --start 0 0 0 --truth_topic /sim/truth --stop_file '$STOP' --max_sim_s 1500" > $OUT/isaac_stdout.txt 2>&1 8>&- 9>&- & SCP=$!
for i in $(seq 240); do grep -q '\[slam\] start' $OUT/isaac_stdout.txt 2>/dev/null && break; kill -0 $SCP 2>/dev/null || break; sleep 1; done
echo "scene_ready_after_s $i" > $OUT/exit_codes.txt
timeout -s INT 1200 /usr/bin/python3 $HERE/probe_turn/probe_turn_moving.py --out $OUT/probe_turn.json $EXTRA > $OUT/probe.log 2>&1; echo "probe_exit $?" >> $OUT/exit_codes.txt
touch $STOP
wait $SCP; echo "isaac_exit $?" >> $OUT/exit_codes.txt
echo "[probe-turn] 끝 $NAME" | tee $OUT/done.txt
