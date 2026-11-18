#!/bin/bash
# 고친 loc_monitor 확인: ① 2 코어 제한 3 판(재현 조건) ② 제한 없음 1 판(PC 기준 판과 비교)
H=$(cd "$(dirname "$0")" && pwd)
NO_CAMS=1 ROS_DOMAIN_ID=78 taskset -c 0,1 bash $H/run_load2.sh 3 > $H/run_2core.log 2>&1
NO_CAMS=1 ROS_DOMAIN_ID=78 bash $H/run_load2.sh 1 > $H/run_full.log 2>&1
echo done > $H/fix_done.txt
