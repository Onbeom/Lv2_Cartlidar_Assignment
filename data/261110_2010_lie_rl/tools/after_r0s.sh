#!/usr/bin/env bash
# 이미 돌고 있는 R0s (pid $1) 끝 -> 요약 -> r2c (r2b model_250 이어 750 반복)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
echo "$(date '+%F %T') R0s 시작 (세운 퍽 기준 512, 옆 퍽 0, pid $1)" | tee -a "$SUM"
tail --pid="$1" -f /dev/null
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0s 끝" | tee -a "$SUM"
grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N|A dock: 쥐는" "$OUT/r0_R0s.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0s.log" | tee -a "$SUM"
CK="$MAN/logs/rsl_rl/waffle_lieupright/2026-11-10_23-48-29/model_250.pt"
TASK=Waffle-LieUprightB2-Direct bash "$W/tools/run_p1_round.sh" r2c "$CK" 750
