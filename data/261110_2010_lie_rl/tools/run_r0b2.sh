#!/usr/bin/env bash
# R0b' (모의, 기록): R0b 와 같되 세울 자리 고르기를 '옆 퍽이 있었다면 놓였을 자리' 기준으로 (--lie_spot_virtual 1, 기본값).
#   bash run_r0b2.sh [기다릴 pid]   (1 회차 연결 스크립트가 끝난 뒤 GPU 잠금을 잡음)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
[ -n "${1:-}" ] && { echo "$(date '+%F %T') R0b' : pid $1 끝 기다림" | tee -a "$SUM"; tail --pid="$1" -f /dev/null; }
exec 9>/tmp/lv2_gpu.lock
if ! flock -w 10800 9; then echo "$(date '+%F %T') R0b' 잠금 대기 3 시간 초과, 건너뜀" | tee -a "$SUM"; exit 2; fi
cd "$MAN"
lg="$OUT/r0_R0b2.log"
echo "$(date '+%F %T') R0b' 시작 (대 512, 옆 퍽 0.0, 가상 자리)" | tee -a "$SUM"
timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_spot_virtual 1 --est --seconds 60 > "$lg" 2>&1
rc=$?; pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0b' 끝 (exit $rc)" | tee -a "$SUM"
grep -aE "mission 배치|경로 계산|끝 분류|A \(세워 내려놓고|다시 집기:|\*\*성공|팔-몸체 1 N|A dock: 쥐는" "$lg" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -aiE -m3 -A3 "Traceback|out of memory" "$lg" | tee -a "$SUM"
