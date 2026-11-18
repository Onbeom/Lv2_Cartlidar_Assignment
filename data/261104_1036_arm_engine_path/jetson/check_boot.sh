#!/usr/bin/env bash
# Jetson 기동 확인: 02bd141(PR #100) 인지 빌드 → 기본 설정으로 노드 띄움 → '검출기:' 줄 · FATAL · 엔진 WARN 확인
#   실행 (Jetson): bash ~/arm_engine_check/check_boot.sh   (같은 폴더에 src_02bd141.tar.gz)
set -u
H=$HOME/arm_engine_check; cd $H || exit 1
E=/home/cart11/yolo_D/deploy
rm -rf workspace build install log; tar xzf src_02bd141.tar.gz
{ echo "== 엔진 sha256 (기대 앞자리 480 58df0823 · 384 e22717e8)"; sha256sum $E/yolo11n_task4_D_480_fp16.engine $E/yolo11n_task4_D_384_fp16.engine; } 2>&1 | tee result.txt
ROS=$(ls -d /opt/ros/*/ 2>/dev/null | head -1); echo "== ROS: $ROS" | tee -a result.txt
set +u; source ${ROS}setup.bash; set -u
colcon build --base-paths workspace/ros2_ws/src --packages-up-to perception --cmake-args -DCMAKE_BUILD_TYPE=Release > build.log 2>&1
echo "== build_rc=$?" | tee -a result.txt
grep -rh "perception: TensorRT" log/latest_build/perception/ 2>/dev/null | head -2 | tee -a result.txt
set +u; source install/setup.bash; set -u
ROS_DOMAIN_ID=139 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST timeout -s INT 30 \
  ros2 launch perception perception.launch.py use_survey:=false use_obstacle_stop:=false use_rear_obstacle:=false > boot.log 2>&1
{ echo "== 검출기 줄"; grep -o "검출기: .*" boot.log; echo "== FATAL 수: $(grep -c FATAL boot.log)"; echo "== 엔진 WARN"; grep -E "engine_path|색 검출기로 대신" boot.log; echo "== 끝"; } | tee -a result.txt
