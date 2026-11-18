#!/usr/bin/env bash
# 2026-11-06 '칸에 못 앉음' 원인 확인: 손 안 퍽 위치를 가정 (지금) 하느냐, 재느냐 (손목 카메라, 잡음 0 · 1 · 2 mm) 에 따라 상자당 2 개 성공이 어떻게 바뀌는지.
# 원기둥 d1 · 직육면체 d9, 48 대 (짝 칸 6 가지 x 8), 앞 칸 다 참. 요약 seat_summary.log, 로그 seat_<이름>.log.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/seat_summary.log"
cleanup() { pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
ev() {  # ev <이름> [인자...]
  local name=$1; shift
  echo "[$(date '+%T')] $name: $*" | tee -a "$SUM"
  timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 "$@" > "seat_$name.log" 2>&1; cleanup
  grep -aE "다 채운 대|끝 분류|손 안 퍽 가정 오차" "seat_$name.log" | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"
}
CYL="--sidepick $HERE/policies/sidepick45_d1.pt"
BOX="--box_prob 1.0 --sidepick $HERE/policies/sidepick45box_d9.pt"
echo "===== $(date '+%F %T') 시작 =====" | tee -a "$SUM"
ev cyl_assume $CYL
ev cyl_meas0 $CYL --est_meas 0
ev cyl_meas1 $CYL --est_meas 1
ev cyl_meas2 $CYL --est_meas 2
ev box_assume $BOX
ev box_meas0 $BOX --est_meas 0
ev box_meas1 $BOX --est_meas 1
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
