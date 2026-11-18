#!/usr/bin/env bash
# 2026-11-06 손 안 퍽 높이를 '닫는 순간 상자 윗면 + 30 mm' 로 알기 (--est_z box, 카메라 · 학습 없음) vs 예전 가정, fit1 정책, 48 대 상자당 2 개.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/estz_summary.log"
KEYS='다 채운 대|끝 분류|손 안 퍽 가정 오차 \|mm\||팔끼리 충돌'
cleanup() { pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
ev() { local name=$1; shift; echo "[$(date '+%T')] $name: $*" | tee -a "$SUM"
  timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 "$@" > "estz_$name.log" 2>&1; cleanup
  grep -aE "$KEYS" "estz_$name.log" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"; }
CYL="--sidepick $HERE/policies/sidepick45_fit1.pt"; BOX="--box_prob 1.0 --sidepick $HERE/policies/sidepick45box_fit1.pt"
echo "===== $(date '+%F %T') 시작 =====" | tee -a "$SUM"
ev cyl_grasp $CYL
ev cyl_box $CYL --est_z box
ev box_grasp $BOX
ev box_box $BOX --est_z box
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
