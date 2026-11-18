#!/usr/bin/env bash
# R0s-t (모의): 앞 사슬 (pid $1) 끝 -> R0s 와 같은데 --est 없이 512
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"; MAN="$W/ws/urdf_and_usd/policy/manipulator"; OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
[ -n "${1:-}" ] && tail --pid="$1" -f /dev/null
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9 || exit 2
cd "$MAN"
echo "$(date '+%F %T') R0s-t 시작 (R0s 와 같음, --est 없음, 512)" | tee -a "$SUM"
timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --seconds 60 > "$OUT/r0_R0s_noest.log" 2>&1
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0s-t 끝" | tee -a "$SUM"
grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N|칸별" "$OUT/r0_R0s_noest.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0s_noest.log" | tee -a "$SUM"
