#!/usr/bin/env bash
# teleop 처럼 팔 스킬 하나씩 (2026-11-13). 시뮬 · 실물 같은 명령:
#   arm_cmd.sh <sim|real> pick                       상자 위 퍽 45° 집기 (toppick_tilt — 실물은 노드 설정 skill.toppick_tilt_policy 의 정책)
#   arm_cmd.sh <sim|real> load <열> <칸>              쥔 퍽을 팔레트 칸에 (pallet_load_<열>_<칸>_tilt), 예: load 3 1
#   arm_cmd.sh <sim|real> unload <열> <칸>            칸에서 꺼냄 (pallet_unload_<열>_<칸>_tilt)
#   arm_cmd.sh <sim|real> place <x> <y> <z>           쥔 퍽을 base_link (x, y) 에, z = 놓을 윗면 높이 (place_at:base_link:x:y:z)
#   arm_cmd.sh real look <x> <y> <z>                 실물: 퍽 예상 중심 (base_link) 을 팔 카메라로 내려다봄 (target_look, pick 전에 필요)
#   arm_cmd.sh real place_cell <L> <dx> <dy>          실물: 장소 L 칸 (dx, dy = m1 · 0 · p1) 에 놓기 (place_<L>_<dx>_<dy>_tilt, 지도 TF 필요)
#   arm_cmd.sh <sim|real> raw <스킬 이름>              그대로 보냄
#   arm_cmd.sh real pose <이름>                     실물: 이름 자세로 (/manipulator/pose, DRIVE · TRANSIT_TILT · STOW …) — 스킬 실패로 팔이 멈춰 있을 때 되돌리기
#   시뮬 전용: transit · reset · cells (상자 위 3x3 칸 좌표) · status (퍽 자리) · clear (상자 위 퍽 치움) · fill <열_칸> ... (팔레트에 퍽 바로 채우기, 예 fill 3_0 3_1 2_1)
# 실물 (real) 은 젯슨에서 (bringup skill_enabled:=true 상태): ros2 action send_goal /manipulator/execute_skill
set -u
MODE="${1:-}"; shift || true; C="${1:-}"; shift || true
case "$C" in
  pick) S="toppick_tilt" ;;
  load|unload)   # 팔레트 v5 (11-13) 10 칸: 1 · 2 열 0 ~ 3, 3 열 0 · 1. PALLET_REMOVED 칸도 거부
    case "${1}_${2}" in 1_[0-3]|2_[0-3]|3_[01]) ;; *) echo "[arm_cmd] 칸 ${1}_${2} 는 팔레트 v5 에 없음 (1 · 2 열 0 ~ 3, 3 열 0 · 1)"; exit 2;; esac
    case " ${PALLET_REMOVED-} " in *" ${1}_${2} "*) echo "[arm_cmd] 칸 ${1}_${2} 는 쓰지 않는 칸 (PALLET_REMOVED='${PALLET_REMOVED-}')"; exit 2;; esac
    S="pallet_${C}_${1}_${2}_tilt" ;;
  place) S="place_at:base_link:${1}:${2}:${3}" ;;
  look) S="target_look:base_link:${1}:${2}:${3}" ;;
  place_cell) S="place_${1}_${2}_${3}_tilt" ;;
  raw) S="$1" ;;
  pose) S="$1" ;;
  transit|reset|cells|status|clear) S="$C" ;;
  fill) S="fill $*" ;;
  *) sed -n 2,13p "$0"; exit 1 ;;
esac
T="${ARM_TIMEOUT:-120}"
if [ "$MODE" = sim ]; then
  set +u; source "$HOME/ros2_lyrical/install/setup.bash" 2>/dev/null; set -u
  exec python3 "$(cd "$(dirname "$0")" && pwd)/sim_cmd.py" "$S" "$T"
elif [ "$MODE" = real ]; then
  # 새 터미널에서 작업 공간을 안 불렀으면 robot_interfaces 를 몰라 "The passed action type is invalid" (11-13) -> 기본 경로로 부름
  ROS_WS="${ROS_WS:-$HOME/Lv2_Cartlidar_Assignment/workspace/ros2_ws}"
  set +u; [ -f "$ROS_WS/install/setup.bash" ] && source "$ROS_WS/install/setup.bash"; set -u
  if [ "$C" = pick ] || [ "$C" = look ]; then ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: 4}" > /dev/null 2>&1 || true; fi   # 인식 켜기 (#190 실물 시험 메모)
  # ros2 action send_goal 은 결과 success: false 여도 끝 코드 0 -> 결과 줄로 판정 (dice.sh 가 실패를 성공으로 세던 문제, 11-13)
  OUT=$(mktemp)
  if [ "$C" = pose ]; then
    ros2 action send_goal /manipulator/pose robot_interfaces/action/ArmPose "{name: '$S'}" 2>&1 | tee "$OUT"
  else
    ros2 action send_goal /manipulator/execute_skill robot_interfaces/action/ExecuteSkill "{policy: '$S', timeout: $T}" --feedback 2>&1 | tee "$OUT"
  fi
  grep -q "success: true" "$OUT"; RC=$?; rm -f "$OUT"
  [ $RC = 0 ] || echo "[arm_cmd] 실패: $S"
  exit $RC
else
  sed -n 2,13p "$0"; exit 1
fi
