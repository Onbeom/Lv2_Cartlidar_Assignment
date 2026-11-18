#!/bin/bash
# 2 코어 제한 1 판 + 재생 60 s 뒤 loc_monitor 스레드별 CPU · 스택 3 번
HERE=$(cd "$(dirname "$0")/.." && pwd)
export PYTHONPATH=$HERE/diag/pyhook${PYTHONPATH:+:$PYTHONPATH}
NO_CAMS=1 ROS_DOMAIN_ID=78 taskset -c 0,1 bash $HERE/run_load2.sh 1 > $HERE/diag/run.log 2>&1 &
sleep 80
P=$(pgrep -f "lib/localization/loc_monitor" | head -1); echo "pid $P" > $HERE/diag/threads.txt
snap() { for t in /proc/$P/task/*; do echo "$(basename $t) $(cut -d' ' -f14,15 $t/stat)"; done; }
snap > $HERE/diag/t0.txt; sleep 5; snap > $HERE/diag/t1.txt
join $HERE/diag/t0.txt $HERE/diag/t1.txt | awk '{printf "tid %s cpu %.1f%%\n",$1,(($4+$5)-($2+$3))/5}' >> $HERE/diag/threads.txt
for i in 1 2 3; do kill -USR1 $P; sleep 1; done
wait
