#!/usr/bin/env bash
# Jetson 기동 확인: 8fcf9f8(PR #103, E2) 인지 빌드 → 기본 설정으로 노드 띄움 → '검출기:' 줄 · FATAL · 엔진 WARN 확인
#   실행 (Jetson): bash ~/e2_engine_check/check_boot.sh   (같은 폴더에 src_8fcf9f8.tar.gz)
set -u
H=$HOME/e2_engine_check; cd $H || exit 1
E=/home/cart11/yolo_D/deploy
rm -rf workspace build install log; tar xzf src_8fcf9f8.tar.gz
{ echo "== 엔진 sha256 (기대 앞자리 480 315b0608 · 384 15d8bb9c)"; sha256sum $E/yolo11n_task4_E2_480_fp16.engine $E/yolo11n_task4_E2_384_fp16.engine; } 2>&1 | tee result.txt
ROS=$(ls -d /opt/ros/*/ 2>/dev/null | head -1); echo "== ROS: $ROS" | tee -a result.txt
set +u; source ${ROS}setup.bash; set -u
colcon build --base-paths workspace/ros2_ws/src --packages-up-to perception --cmake-args -DCMAKE_BUILD_TYPE=Release > build.log 2>&1
echo "== build_rc=$?" | tee -a result.txt
grep -rh "perception: TensorRT" log/latest_build/perception/ 2>/dev/null | head -2 | tee -a result.txt
set +u; source install/setup.bash; set -u
ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout -s INT 30 \
  ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > boot.log 2>&1
{ echo "== 검출기 줄"; grep -o "검출기: .*" boot.log; echo "== FATAL 수: $(grep -c FATAL boot.log)"; echo "== 엔진 WARN"; grep -E "engine_path|색 검출기로 대신" boot.log; echo "== 끝"; } | tee -a result.txt
