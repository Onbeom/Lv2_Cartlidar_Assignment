#!/usr/bin/env bash
# run_after_r0.sh (pid $1) 끝 -> P1 도구 확인에 Traceback 없고 시작 상태 파일이 있으면 1 회차 (2048 x 1000) 시작
W="$(cd "$(dirname "$0")/.." && pwd)"
tail --pid="$1" -f /dev/null
if grep -aq Traceback "$W/results/p1_smoke.log" || ! grep -aq "Learning iteration" "$W/results/p1_smoke.log"; then
  echo "$(date '+%F %T') 도구 확인 실패 -> 1 회차 안 함" | tee -a "$W/results/p1_rounds.txt"; exit 1; fi
bash "$W/tools/run_p1_round.sh" r1
