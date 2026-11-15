#!/usr/bin/env bash
# Jetson 라이다만 노트북 RViz2 로 (ssh 로 받아 노트북 localhost ROS 에 다시 냄 — Wi-Fi DDS 안 씀, 로봇 쪽은 읽기만)
#   ./scan_view.sh                 그대로
#   ./scan_view.sh front|rear|both 그 라이다를 좌우 뒤집어 보기 (장착 방향 확인용)
#   ./scan_view.sh amcl            로봇 위치 추정 (AMCL) 자세를 따라 그림 — 시연 전 · 중 감시 (11-15)
#   ./scan_view.sh rot             앞 · 뒤 라이다를 자기 축으로 180° 돌려 보기 → 초록 · 파랑 (scan_merged 빨강은 그대로)
# RViz: 빨강 = /scan_merged, 초록 · 파랑 = 앞 · 뒤 (Displays 에서 켜기), 초록 선 = 차체. 위 도구 "2D Pose Estimate" 로 로봇 자리 · 방향을 끌어 옮기면
#   스캔이 지도에 겹치는 자리를 찾을 수 있음 (터미널에 그 좌표가 나옴). 처음 = 계획 출발 (0.694, 0.950, 180°)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
set +u; source /opt/ros/lyrical/setup.bash; set -u
export ROS_DOMAIN_ID=74 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 11-15: 다른 PC 면 MAP=<저장소>/workspace/ros2_ws/src/mission_planner/maps/map_amcl.yaml · JETSON=<사용자@주소> 로 바꿔 실행
MAP="${MAP:-$HERE/../../../ros2_ws/src/mission_planner/maps/map_amcl.yaml}"   # 저장소 workspace/tools/drive_demo/scan_view 기준
[ -f "$MAP" ] || MAP="$HOME/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/mission_planner/maps/map_amcl.yaml"   # 바탕화면 판
JETSON="${JETSON:-cart11@cart11.local}"
[ -f "$MAP" ] || { echo "✗ 지도 파일 없음: $MAP  (MAP=… 로 저장소의 mission_planner/maps/map_amcl.yaml 경로를 주세요)"; exit 1; }
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; sleep 1; for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" > /tmp/scan_view_map.log 2>&1 & PIDS+=($!)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lm_scan_view -p autostart:=true -p node_names:="['map_server']" > /tmp/scan_view_lm.log 2>&1 & PIDS+=($!)
python3 "$HERE/scan_relay.py" "$JETSON" "${1:-}" & PIDS+=($!)
rviz2 -d "$HERE/scan_view.rviz"
