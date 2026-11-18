#!/usr/bin/env bash
# P2b 손-눈 보정 촬영 (Jetson, ssh): Jetson 모니터에 보정판을 띄우고 arm_handeye.py 실행. 인자는 그대로 넘김 (예: --out ~/real_drive/handeye/run1)
#   모니터에 cart11 로 로그인해 있어야 창이 뜬다 (arm_calib.sh 와 같은 방식).
U=$(id -u); D=""
for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && [ "$(stat -c %u "$s")" = "$U" ] && { D=":${s##*/X}"; break; }; done
[ -z "$D" ] && for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && { D=":${s##*/X}"; break; }; done
A=""
for x in /run/user/$U/gdm/Xauthority /run/user/$U/.mutter-Xwaylandauth.* /run/user/$U/xauth_* $HOME/.Xauthority; do [ -f "$x" ] && { A=$x; break; }; done
echo "화면: DISPLAY=${D:-없음} XAUTHORITY=${A:-없음}"
[ -n "$D" ] && export DISPLAY=$D; [ -n "$A" ] && export XAUTHORITY=$A
cd "$HOME/real_drive" || exit 1
# 창을 띄울 수 있는지 먼저 시험 (모니터가 로그인 화면이면 Qt 가 프로그램을 통째로 끔) → 못 띄우면 보드 없이 진행
if timeout 10 python3 -c "import cv2; cv2.namedWindow('t'); cv2.waitKey(50); cv2.destroyAllWindows()" >/dev/null 2>&1; then
  exec python3 arm_handeye.py --show_board "$@"
fi
echo "Jetson 모니터에 창을 못 띄움 (모니터에 cart11 로 로그인 안 됨?) → 보드를 다른 화면(PC 등)에 띄우고 진행합니다"
exec python3 arm_handeye.py "$@"
