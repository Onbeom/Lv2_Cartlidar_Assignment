# 이 시험에서 띄운 ros2 bag record 를 모두 끔 (SIGINT → 6 s 뒤 남으면 SIGTERM). 이름 비교는 /proc 의 명령줄로 해서 자기 셸은 안 건드림
list() { for d in /proc/[0-9]*; do p=${d#/proc/}; [ "$p" = "$$" ] && continue; c=$(tr '\0' ' ' < $d/cmdline 2>/dev/null); case "$c" in *"bin/ros2 bag record"*) echo $p;; esac; done; }
P=$(list); [ -n "$P" ] && kill -INT $P; sleep 6
P=$(list); [ -n "$P" ] && { kill -TERM $P; sleep 3; }
P=$(list); [ -n "$P" ] && { echo "남음: $P"; exit 1; }
echo "기록기 없음"
