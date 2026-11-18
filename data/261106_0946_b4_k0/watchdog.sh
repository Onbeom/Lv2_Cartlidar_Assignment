#!/bin/bash
# 11-06 Isaac 이 죽었는데 run_goals 가 3600 s 제한까지 기다리는 것을 막음: 장면이 30 s 넘게 없는데 run_goals 가 살아 있으면 run_goals 만 끝냄.
# 그 뒤 run_b4.sh 가 isaac_exit 139 를 보고 _TOOLFAIL 로 바꿔 다시 돌린다. results/done.txt 가 나오면 끝.
R=/home/pa4/git/lv2_next_module/261106_0946_b4_k0/results
miss=0
while [ ! -e "$R/done.txt" ]; do
  if pgrep -f "test/run_goals[.]py .*$R" >/dev/null && ! pgrep -f "scene_live[.]py.*$R" >/dev/null; then
    miss=$((miss + 1))
  else
    miss=0
  fi
  if [ "$miss" -ge 6 ]; then
    echo "$(date +%H:%M:%S) 장면 없음 → run_goals 끝냄" >> "$R/watchdog.log"
    pkill -INT -f "test/run_goals[.]py .*$R"
    miss=0
  fi
  sleep 5
done
