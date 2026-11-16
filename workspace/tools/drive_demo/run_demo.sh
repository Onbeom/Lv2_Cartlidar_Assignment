#!/usr/bin/env bash
# 규칙 v2 계획 시연 — Nav2 (controller_server RPP · behavior_server Spin) + 규칙 v2 필터 + 계획 따라가기. 저장소 파일은 안 고침.
#   ./run_demo.sh real [slow|full] [step] [init] [구간 예 0-0]     Jetson (로봇): 로봇 쪽 launch 3 개가 떠 있어야 함 (시연_가이드.md)
#   ./run_demo.sh sim  [slow|full] [step] [구간]                   노트북: loopback 시뮬로 같은 것을 미리 확인 (Gazebo 없음)
#     slow = 전진 0.20 · 후진 0.10 m/s · 제자리 0.4 rad/s (첫 실물 권장)   full = 0.30 · 0.15 · 0.5 (시뮬 18:46 과 같음)
#     step = 구간마다 Enter 로 출발 (q = 끝)   init = 출발 자세를 /initialpose 로 한 번 (로봇을 출발 자리에 정확히 놓았을 때)
#     구간 = 그 구간만 (예 0-0 = 첫 구간 signal → box1 만, 2-4)
# 멈추기: 이 터미널 Ctrl+C (Nav2 동작 취소 → 0) · 리모컨 1 (E-stop, 풀기 2+3) — 소프트웨어 정지, 물리 비상정지 아님
# 기록: 이 폴더 runs/<월일_시각>_<실물|시뮬>_<speed>/ → 노트북 2026-11-12_기준주행/runs/ 로 복사하면 주행기록 HTML 에 Nav2 기록으로 보임
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
BELT="${BELT:-}"; TURN_MOTOR="${TURN_MOTOR:-}"   # 11-15: 필터의 벨트 기어비 · 꺾는 중 모터 상한 (빈 값 = 1.5 · 3.0, 예 BELT=2.0 TURN_MOTOR=2.6)
PLAN="${PLAN:-$HERE/demo_plan.json}"   # 11-15: 다른 계획 시험 (PLAN=… ./run_demo.sh sim slow) · PARAMS= 로 Nav2 설정 · SIM_DOMAIN= 로 시뮬 ROS_DOMAIN_ID
MODE=""; SPEED=slow; STEP=""; INIT=""; LEGS=""
for a in "$@"; do
  case "$a" in
    real|sim) MODE=$a ;; slow|full) SPEED=$a ;; step) STEP=--step ;; init) INIT=--set-initial ;;
    [0-9]*) LEGS=$a ;; *) echo "모르는 인자: $a"; exit 2 ;;
  esac
done
[ -n "$MODE" ] || { sed -n 2,9p "$0"; exit 2; }
if [ "$SPEED" = full ]; then VF=0.30; VR=0.15; SW=0.5; else VF=0.20; VR=0.10; SW=0.4; fi
set +u; source /opt/ros/lyrical/setup.bash; set -u
for d in "$(cd "$HERE/../../.." 2>/dev/null && pwd)" "$HOME/Lv2_Cartlidar_Assignment" "$HOME/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment"; do   # 첫 후보 = 이 폴더가 든 저장소 (workspace/tools/drive_demo)
  [ -f "$d/workspace/ros2_ws/install/setup.bash" ] && { set +u; source "$d/workspace/ros2_ws/install/setup.bash"; set -u; REPO=$d; break; }
done
OUT="$HERE/runs/$(date +%m%d_%H%M)_$([ $MODE = real ] && echo 실물 || echo 시뮬)_$SPEED${BELT:+_벨트$BELT}${TURN_MOTOR:+_모터$TURN_MOTOR}${LEGS:+_구간$LEGS}${RUN_TAG:+_$RUN_TAG}"   # 11-15: RUN_TAG= (같은 분에 여러 시뮬)
mkdir -p "$OUT"
LOG="$OUT/nav.log"
ODOM=/odometry/filtered; SIMP=(); OUTT=cmd_vel_nav
if [ $MODE = sim ]; then
  export ROS_DOMAIN_ID=${SIM_DOMAIN:-73} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ODOM=/odom; SIMP=(-p use_sim_time:=true); OUTT=cmd_vel
else   # 실물: 로봇 쪽과 같은 ROS 설정 그대로. 이미 Nav2 가 떠 있으면 이름이 겹침
  if ros2 node list 2>/dev/null | grep -qE "^/(controller_server|behavior_server|v2_filter)$"; then
    echo "✗ controller_server · behavior_server · v2_filter 가 이미 있음 (bringup use_nav2:=false 인지, 이 스크립트가 두 번 떴는지 확인)"; exit 1; fi
  ros2 topic info /cmd_vel_nav 2>/dev/null | grep -q "Subscription count: [1-9]" || echo "⚠ /cmd_vel_nav 를 받는 쪽 (twist_mux) 이 안 보임 — base_bringup 이 떠 있는지 (터미널 1)"
fi
sed -e "s/__VF__/$VF/g" -e "s/__VR__/$VR/g" -e "s#__ODOM__#$ODOM#g" "${PARAMS:-$HERE/params_demo.yaml}" > "$OUT/params.yml"
PIDS=()
start() { setsid "$@" >> "$LOG" 2>&1 < /dev/null & PIDS+=($!); }
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- -"$p" 2>/dev/null; done; }
trap cleanup EXIT TERM

echo "규칙 v2 시연 ($MODE · $SPEED: 전진 $VF · 후진 $VR m/s · 제자리 $SW rad/s) · 기록 $OUT" | tee -a "$LOG"
NODES="'controller_server', 'behavior_server'"
if [ $MODE = sim ]; then
  MAP="${SIM_MAP:-$REPO/workspace/ros2_ws/src/mission_planner/maps/map_amcl.yaml}"   # 11-15: SIM_MAP= 으로 다른 지도 (예 실측판)
  start ros2 run nav2_map_server map_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -p yaml_filename:="$MAP"
  start ros2 run nav2_loopback_sim loopback_simulator --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}"
  start ros2 run tf2_ros static_transform_publisher --x 0.0 --y 0.0 --z 0.0755 --frame-id base_link --child-frame-id base_scan
  NODES="'map_server', 'loopback_simulator', $NODES"
fi
start ros2 run nav2_controller controller_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
start ros2 run nav2_behaviors behavior_server --ros-args --params-file "$OUT/params.yml" "${SIMP[@]}" -r cmd_vel:=cmd_vel_raw
start python3 "$HERE/v2_filter.py" --ros-args "${SIMP[@]}" -p out_topic:=$OUTT -p vx_fwd:=$VF -p vx_rev:=$VR -p spin_w:=$SW \
  -p belt:=$(printf %.3f "${BELT:-1.5}") -p turn_motor:=$(printf %.3f "${TURN_MOTOR:-3.0}") -p odom_topic:=$ODOM -p rev_max:=$(printf %.3f "${REV_MAX:-$([ $MODE = real ] && echo 0.50 || echo 0.40)}")
start ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=lifecycle_manager_demo "${SIMP[@]}" \
  -p autostart:=true -p bond_timeout:=0.0 -p node_names:="[$NODES]"

{ echo "이름: 계획따라_$([ $MODE = real ] && echo 실물 || echo 시뮬)_$SPEED${BELT:+_벨트$BELT}${TURN_MOTOR:+_모터$TURN_MOTOR}${LEGS:+_구간$LEGS}${RUN_TAG:+_$RUN_TAG}"; echo "시각: $(date '+%F %T')"
  echo "경로: $(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['course'])" "$PLAN")"; echo "GUI=0"
  echo "추가 인자: $([ $MODE = real ] && echo '실물 (AMCL · EKF)' || echo 'loopback 시뮬') · 계획 $(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['plan'])" "$PLAN") · RPP 전진 $VF · 후진 $VR · Spin · 규칙 v2 필터 (제자리 ≤ $SW · 벨트 ${BELT:-1.5} · 꺾는 중 모터 ≤ ${TURN_MOTOR:-3.0} · 이어진 후진 ≤ ${REV_MAX:-$([ $MODE = real ] && echo 0.50 || echo 0.40)} m)${PARAMS:+ · Nav2 설정 $(basename "$PARAMS")}"
  [ $MODE = real ] && echo "실물 벨트: ${HW_BELT:-2.0} (11-15 v7 44T → 22T, 다르면 HW_BELT=)"
  echo "저장소: Nav2 노드만 (시연 폴더 $(basename "$HERE"))"; } > "$OUT/settings.txt"

sleep 3
# 11-15 21:0x: Jetson 에서 lifecycle 응답이 시간 초과로 사라져 서버가 inactive 로 남는 일이 잦음 (map_server · behavior_server, #276 같은 DDS 지연)
#   → 출발 전에 직접 확인하고 unconfigured 면 configure, inactive 면 activate (로봇은 움직이지 않음 — 서버만 켬)
ensure_active() {
  local n st k
  for n in "$@"; do
    for k in $(seq 1 25); do
      st=$(timeout 8 ros2 lifecycle get "/$n" 2>/dev/null | awk '{print $1}')
      case "$st" in
        active) echo "  ✓ $n active" | tee -a "$LOG"; break ;;
        unconfigured) timeout 20 ros2 lifecycle set "/$n" configure >> "$LOG" 2>&1 ;;
        inactive) timeout 20 ros2 lifecycle set "/$n" activate >> "$LOG" 2>&1 ;;
        *) sleep 1 ;;
      esac
      if [ "$k" = 25 ]; then echo "  ✗ $n 이 active 가 안 됨 (지금 ${st:-응답 없음}) — 다른 터미널에서 ros2 lifecycle set /$n activate" | tee -a "$LOG"; fi
    done
  done
}
if [ $MODE = sim ]; then ensure_active map_server loopback_simulator controller_server behavior_server
else ensure_active map_server controller_server behavior_server; fi
python3 "$HERE/demo_follow.py" --plan "$PLAN" --out "$OUT" $([ $MODE = sim ] && echo --sim) $STEP $INIT ${LEGS:+--legs "$LEGS"} ${OFF_PATH:+--off-path "$OFF_PATH"} 2>&1 | (trap '' INT; tee "$OUT/follow.log")   # 11-15: Ctrl+C 로 tee 가 먼저 죽으면 demo_follow 가 BrokenPipe 로 저장 못 함
cleanup; trap - EXIT
grep -h "잘림 횟수" "$LOG" | tail -1
echo "기록: $OUT"
