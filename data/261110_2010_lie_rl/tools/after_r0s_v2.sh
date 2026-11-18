#!/usr/bin/env bash
# R0s (pid $1) 끝 -> 요약 -> 칸에 못 앉음 비율이 R0b' 13.4 % ± 5 %p 이면 R0t (세운 퍽 자리 기준 512) -> r2c (r2b model_250 이어 750)
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
MAN="$W/ws/urdf_and_usd/policy/manipulator"
OUT="$W/results"
export ISAACLAB_DIR="$HOME/pa-course/IsaacLab-6.1"
SUM="$OUT/r0_summary.txt"
tail --pid="$1" -f /dev/null
pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
echo "$(date '+%F %T') R0s 끝" | tee -a "$SUM"
grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N" "$OUT/r0_R0s.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0s.log" | tee -a "$SUM"
R=$(python3 - "$OUT/r0_R0s.log" <<'PY'
import re,sys,ast
t=open(sys.argv[1],errors="ignore").read()
m=re.search(r"세운 퍽 기준: .*? (\d+)/(\d+) 대",t); c=re.search(r"끝 분류 \(대 \d+\): (\{.*\})",t)
if not (m and c): print("nan"); sys.exit()
d=ast.literal_eval(c.group(1)); st=int(m.group(1))
print(f"{d.get('칸에 못 앉음',0)/max(1,st)*100:.1f}")
PY
)
echo "$(date '+%F %T') R0s 칸에 못 앉음 / 시작한 대 = $R %" | tee -a "$SUM"
if python3 -c "import sys; r=float('$R'); sys.exit(0 if abs(r-13.4)<=5 else 1)"; then
  (
  exec 9>/tmp/lv2_gpu.lock
  flock -w 3600 9 || exit 2
  cd "$MAN"
  echo "$(date '+%F %T') R0t 8 대 시험" | tee -a "$SUM"
  timeout -k 30s -s INT 20m ./run_rl.sh chain45_eval 8 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --lie_stand_at_nb 1 --est --seconds 60 > "$OUT/r0_R0t_S.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  grep -aE "세운 퍽 기준|끝 분류|\*\*성공" "$OUT/r0_R0t_S.log" | sed 's/^/    /' | cut -c1-300 | tee -a "$SUM"
  grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0t_S.log" | tee -a "$SUM"
  if ! grep -aq "8 단계부터 8/8 대" "$OUT/r0_R0t_S.log"; then echo "$(date '+%F %T') R0t 8 대 시험: 8/8 아님 -> 512 건너뜀" | tee -a "$SUM"; exit 0; fi
  echo "$(date '+%F %T') R0t 시작 (세운 퍽 자리 기준 512)" | tee -a "$SUM"
  timeout -k 30s -s INT 100m ./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --lie_stand_at_nb 1 --est --seconds 60 > "$OUT/r0_R0t.log" 2>&1
  pkill -9 -f "[c]hain45_play.py" 2>/dev/null; sleep 5
  echo "$(date '+%F %T') R0t 끝" | tee -a "$SUM"
  grep -aE "세운 퍽 기준|경로 계산|끝 분류|\*\*성공|팔-몸체 1 N" "$OUT/r0_R0t.log" | sed 's/^/    /' | cut -c1-400 | tee -a "$SUM"
  grep -aiE -m3 -A3 "Traceback|out of memory" "$OUT/r0_R0t.log" | tee -a "$SUM"
  )
else
  echo "$(date '+%F %T') R0s 가 R0b' 와 5 %p 넘게 다름 -> R0t 안 함" | tee -a "$SUM"
fi
CK="$MAN/logs/rsl_rl/waffle_lieupright/2026-11-10_23-48-29/model_250.pt"
TASK=Waffle-LieUprightB2-Direct bash "$W/tools/run_p1_round.sh" r2c "$CK" 750
