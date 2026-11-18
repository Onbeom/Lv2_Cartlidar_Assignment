#!/bin/bash
# 도구 확인(모의, 판정 아님): 빈 바닥(empty_maze.usda)에 scene_live.py(cmd_vel 모드)를 띄우고 probe_turn_moving.py 로 전진 중 회전 곡선을 잰다.
# 261102 사본: 모델 폴더(LV2_ISAAC_MODEL)를 골라 정해진 명령 순서(probe_cmd.py) → analyze.py. 사용: run_probe.sh <이름> <모델 isaac 폴더>
set -o pipefail
NAME=$1; DOM=102
TOP=/home/pa4/git/lv2_next_module
HERE=/home/pa4/git/lv2_next_module/261102_1450_isaac_B_probe; MODEL=$2
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
sha256sum $HERE/scene_live.py $HERE/probe_cmd.py $HERE/analyze.py $HERE/run_probe.sh $MODEL/robot_params.json $MODEL/urdf/sensors.json > $OUT/source_sha256.txt
STOP=$OUT/stop_scene
env -u AMENT_PREFIX_PATH LV2_ISAAC_MODEL="$MODEL" bash -c "export PYTHONPATH='$PYLINKS':\$PYTHONPATH && exec '$LAB_PY' '$HERE/scene_live.py' --out '$OUT/scene' --light_level 1.0 --mode cmd_vel \
    --maze '$TOP/261101_0016_isaac_live/probe_turn/empty_maze.usda' --odom_rot_scale 1.0 --start 0 0 0 --truth_topic /sim/truth --stop_file '$STOP' --max_sim_s 1500" > $OUT/isaac_stdout.txt 2>&1 8>&- 9>&- & SCP=$!
for i in $(seq 240); do grep -q '\[slam\] start' $OUT/isaac_stdout.txt 2>/dev/null && break; kill -0 $SCP 2>/dev/null || break; sleep 1; done
echo "scene_ready_after_s $i" > $OUT/exit_codes.txt
timeout -s INT 600 /usr/bin/python3 $HERE/probe_cmd.py > $OUT/probe.log 2>&1; echo "probe_exit $?" >> $OUT/exit_codes.txt
touch $STOP
wait $SCP; echo "isaac_exit $?" >> $OUT/exit_codes.txt
python3 $HERE/analyze.py $OUT/scene/truth.csv $OUT/analyze.json > $OUT/analyze.log 2>&1; echo "analyze_exit $?" >> $OUT/exit_codes.txt
echo "[probe] 끝 $NAME" | tee $OUT/done.txt
