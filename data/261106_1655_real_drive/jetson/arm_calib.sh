#!/usr/bin/env bash
# Jetson 모니터에 보정판을 띄우고 ssh 터미널에서 촬영한다. 실행 (Jetson, ssh): bash ~/real_drive/arm_calib.sh
#   모니터에 cart11 로 로그인해 있어야 창이 뜬다. 못 뜨면 --no_board 로 하고 charuco_10x7_5x5_100.png 를 다른 화면에 띄운다.
U=$(id -u); D=""
for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && [ "$(stat -c %u "$s")" = "$U" ] && { D=":${s##*/X}"; break; }; done
[ -z "$D" ] && for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && { D=":${s##*/X}"; break; }; done
A=""
for x in /run/user/$U/gdm/Xauthority /run/user/$U/.mutter-Xwaylandauth.* /run/user/$U/xauth_* $HOME/.Xauthority; do [ -f "$x" ] && { A=$x; break; }; done
echo "화면: DISPLAY=${D:-없음} XAUTHORITY=${A:-없음}"
[ -n "$D" ] && export DISPLAY=$D; [ -n "$A" ] && export XAUTHORITY=$A
exec python3 "$HOME/real_drive/arm_calib_capture.py" "$@"
