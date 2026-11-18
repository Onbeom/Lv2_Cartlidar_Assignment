#!/bin/bash
# Isaac 이 죽었는데 run_goals 가 제한까지 기다리는 것을 막음. results/done.txt 나오면 끝.
R=/home/pa4/git/lv2_next_module/261106_1152_gap_fix/results; miss=0
while [ ! -e "$R/done.txt" ]; do
  if pgrep -f "test/run_goals[.]py .*$R/I" >/dev/null && ! pgrep -f "scene_live[.]py.*$R" >/dev/null; then miss=$((miss + 1)); else miss=0; fi
  if [ "$miss" -ge 6 ]; then echo "$(date +%T) 장면 없음 → run_goals 끝냄" >> "$R/watchdog.log"; pkill -INT -f "test/run_goals[.]py .*$R/I"; miss=0; fi
  sleep 5
done
