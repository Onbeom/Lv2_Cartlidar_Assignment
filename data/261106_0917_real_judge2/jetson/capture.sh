#!/usr/bin/env bash
# Jetson 모니터(DP)에 팔 카메라 화면을 띄우고 ssh 터미널에서 녹화를 켜고 끈다. 실행 (Jetson, ssh): bash ~/real_judge2/capture.sh [capture.py 인자]
#   모니터에 cart11 로 로그인한 화면(X · Xwayland)의 DISPLAY 와 인증 파일을 찾아 쓴다. 바꾸려면 DISPLAY_JETSON=:1 처럼 준다
U=$(id -u)
if [ -n "${DISPLAY_JETSON:-}" ]; then D=$DISPLAY_JETSON
else D=""; for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && [ "$(stat -c %u "$s")" = "$U" ] && { D=":${s##*/X}"; break; }; done
     [ -z "$D" ] && for s in /tmp/.X11-unix/X*; do [ -S "$s" ] && { D=":${s##*/X}"; break; }; done
fi
A=""
for x in /run/user/$U/gdm/Xauthority /run/user/$U/.mutter-Xwaylandauth.* /run/user/$U/xauth_* $HOME/.Xauthority; do [ -f "$x" ] && { A=$x; break; }; done
echo "화면: DISPLAY=${D:-없음} XAUTHORITY=${A:-없음} (X 소켓: $(ls /tmp/.X11-unix 2>/dev/null | tr '\n' ' '))"
[ -z "$D" ] && { echo "모니터 화면(X)을 못 찾음: Jetson 모니터에서 cart11 로 로그인했는지 확인"; exit 1; }
export DISPLAY=$D; [ -n "$A" ] && export XAUTHORITY=$A
exec python3 $HOME/real_judge2/capture.py "$@"
