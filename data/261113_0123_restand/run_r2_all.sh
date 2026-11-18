#!/usr/bin/env bash
# R2 · R3 전부 차례로 (PC). 결과: $1/<경우>/
R=${1:-$(cd "$(dirname "$0")" && pwd)/r2}
H="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$R"
for c in "30 0,0 b30" "-30 0,0 bm30" "45 0,0 b45" "-45 0,0 bm45" "-35 0,0 bm35" "30 0.05,0.05 s30"; do
  set -- $c
  echo "== $3 rot $1 shift $2"
  "$H/run_mock.sh" "$R/$3" true true --start_rot "$1" --start_shift "$2"
done
echo "== navfail"; "$H/run_mock.sh" "$R/navfail" true true --start_rot 30 --start_shift 0,0 --nav_reject_off_table
echo "== r3_off_new (restand false)"; "$H/run_mock.sh" "$R/r3_off_new" true false --start_rot 30 --start_shift 0,0
echo "== r3_off_old (#247)"; OLD=1 "$H/run_mock.sh" "$R/r3_off_old" true false --start_rot 30 --start_shift 0,0
echo "== r3_dis_new (enabled false)"; "$H/run_mock.sh" "$R/r3_dis_new" false false
echo "== r3_dis_old (#247, enabled false)"; OLD=1 "$H/run_mock.sh" "$R/r3_dis_old" false false
