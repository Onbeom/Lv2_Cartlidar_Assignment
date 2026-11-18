#!/usr/bin/env bash
# 최종 확인 영상 촬영. 창 없이 ssh 터미널에서 키로 찍는다.
#   bash final_capture.sh            앞 1~3 → 뒤 1~3 → 팔 1~3 (이미 찍은 회차는 건너뜀, 끊겨도 다시 치면 이어짐)
#   bash final_capture.sh front      앞 1~3 만 (rear · arm 도 같음)
#   bash final_capture.sh front 2 --retake    앞 2 를 다시 찍음(옛것은 video_final_failed 로 옮김)
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv="$here/.venv/bin/python"

# 미리보기 창을 Jetson 모니터(DP)로 보낸다. cart11 로 로그인한 화면을 찾는다(바꾸려면 DISPLAY_JETSON=:1)
uid=$(id -u); screen="${DISPLAY_JETSON:-}"
if [ -z "$screen" ]; then
  for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && [ "$(stat -c %u "$s")" = "$uid" ] && { screen=":${s##*/X}"; break; }; done
  [ -z "$screen" ] && for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && { screen=":${s##*/X}"; break; }; done
fi
auth=""
for x in /run/user/$uid/gdm/Xauthority /run/user/$uid/.mutter-Xwaylandauth.* /run/user/$uid/xauth_* "$HOME/.Xauthority"; do [ -f "$x" ] && { auth=$x; break; }; done
echo "미리보기 화면: DISPLAY=${screen:-없음} XAUTHORITY=${auth:-없음}"
export DISPLAY="$screen"; [ -n "$auth" ] && export XAUTHORITY="$auth"

reset_d435() {
  "$venv" -c "import pyrealsense2 as rs,time; [d.hardware_reset() for d in rs.context().devices]; time.sleep(6); [print('재시작 뒤',d.get_info(rs.camera_info.serial_number),'USB',d.get_info(rs.camera_info.usb_type_descriptor)) for d in rs.context().devices]"
}

one() {  # 카메라 회차 [옵션]
  local cam=$1 round=$2 rc answer py="$venv"; shift 2
  [ "$cam" = arm ] && py=python3   # 팔은 지금까지처럼 시스템 python3 로 찍는다
  while true; do
    "$py" "$here/final_capture.py" "$cam" "$round" "$@"; rc=$?
    case $rc in
      0|2) return 0 ;;
      130) exit 130 ;;
    esac
    read -r -p "실패. Enter: 다시 찍기 · n: 다음 회차로 > " answer
    [ "$answer" = n ] && return 0
    [ "$cam" != arm ] && reset_d435
  done
}

cams=(front rear arm)
case "${1:-}" in
  front|rear|arm) cams=("$1"); shift ;;
  "") ;;
  *) echo "사용: bash final_capture.sh [front|rear|arm] [1|2|3] [--retake]"; exit 2 ;;
esac
rounds=(1 2 3)
case "${1:-}" in 1|2|3) rounds=("$1"); shift ;; esac

df -h "$HOME" | tail -1 | awk '{print "Jetson 디스크 여유 " $4}'
for cam in "${cams[@]}"; do
  for round in "${rounds[@]}"; do
    one "$cam" "$round" "$@"
  done
done
echo "--- 지금까지 찍은 것"
"$venv" "$here/final_capture.py" summary
