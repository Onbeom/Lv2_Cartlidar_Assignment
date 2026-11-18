#!/usr/bin/env bash
# R0 (모의, 기록): 지금 각본 (누운 직육면체 세워 놓기 + n5 다시 잡기 + 12 칸 넣기) 을 미션 배치 (--lie_layout mission) 로.
#   S  : 8 대 시험 (배치 · 코드 확인)
#   R0a: 512 대, 옆 퍽 있음 (지금 각본)
#   R0b: 512 대, 옆 퍽 없음 (서 있는 퍽 먼저 = 순서 바꾸기)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"; mkdir -p "$OUT"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
exec 9>/tmp/lv2_gpu.lock
echo "$(date '+%F %T') GPU 잠금 기다림" | tee -a "$SUM"
if ! flock -w 1800 9; then echo "$(date '+%F %T') 잠금 대기 30 분 초과, 건너뜀" | tee -a "$SUM"; exit 2; fi
echo "$(date '+%F %T') 잠금 잡음" | tee -a "$SUM"
cd "$MAN"
pick() {  # 로그 요약 줄
  grep -aE "mission 배치|경로 계산|끝 분류|A \(세워 내려놓고|다시 집기:|\*\*성공|옆 퍽 닿은 대|팔-몸체 1 N|단계 시작 시각" "$1" | sed 's/^/    /' | cut -c1-400
}
run() {  # $1 이름, $2 대 수, $3 옆 퍽 확률, $4 시간 한도
  local lg="$OUT/r0_$1.log"
  echo "$(date '+%F %T') $1 시작 (대 $2, 옆 퍽 $3)" | tee -a "$SUM"
  timeout -k 30s -s INT "$4" ./run_rl.sh chain45_eval "$2" --lie_regrasp --lie_layout mission --lie_nb_prob "$3" --est --seconds 60 > "$lg" 2>&1
  local rc=$?
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  echo "$(date '+%F %T') $1 끝 (exit $rc)" | tee -a "$SUM"
  pick "$lg" | tee -a "$SUM"
  if grep -aqiE "Traceback|out of memory" "$lg"; then grep -aiE -m3 -A3 "Traceback|out of memory" "$lg" | tee -a "$SUM"; return 1; fi
  return 0
}
run S 8 0.5 20m || { echo "시험 실패, 멈춤" | tee -a "$SUM"; exit 1; }
run R0a 512 1.0 100m
run R0b 512 0.0 100m
echo "$(date '+%F %T') 끝" | tee -a "$SUM"
