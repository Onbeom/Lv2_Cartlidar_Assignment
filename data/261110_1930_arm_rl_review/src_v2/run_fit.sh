#!/usr/bin/env bash
# 2026-11-06 쥐는 깊이 · 높이 일정하게 (rew_grasp_fit −3, 기준 2 mm) 이어 학습 -> 진단 -> 상자당 2 개 평가 (손 안 퍽 '가정', 48 대).
# 비교 기준 (run_seat.sh, 같은 조건): 원기둥 d1 성공 22/48 · 못 앉음 14, 직육면체 d9 성공 12/48 · 못 앉음 22.
#   TRAIN_MIN=30 ./run_fit.sh   (기본 학습 제한 40 분)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/fit_summary.log"
latest_ckpt() { for d in $(ls -d "$HERE/logs/rsl_rl/$1"/*/ | sort -r); do m=$(ls "$d"model_*.pt 2>/dev/null | sort -V | tail -1); [ -n "$m" ] && { echo "$m"; return; }; done; }
TAG="${TAG:-fit1}"  # 정책 이름 policies/*_<TAG>.pt
CK_CYL="${CK_CYL:-$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-05_21-19-00/model_5849.pt}"      # 기본 d1, latest = 가장 최근 학습
CK_BOX="${CK_BOX:-$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-06_01-16-21/model_28291.pt}"  # 기본 d9
[ "$CK_CYL" = latest ] && CK_CYL=$(latest_ckpt waffle_sidepick45)
[ "$CK_BOX" = latest ] && CK_BOX=$(latest_ckpt waffle_sidepick45box)
KEYS='한 번 시도 기준|팔-몸체 \(1 N|다 채운 대|끝 분류|손 안 퍽 가정 오차 \|mm\||팔끼리 충돌'
cleanup() { pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null; pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
show() { grep -aE "$KEYS" "$1" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"; }
echo "===== $(date '+%F %T') 시작 =====" | tee -a "$SUM"
echo "[$(date '+%T')] $TAG 학습 시작 ($CK_CYL | $CK_BOX, 500 반복 동시)" | tee -a "$SUM"
timeout -s INT "${TRAIN_MIN:-40}m" ./run_rl.sh sidepick45 256 500 "$CK_CYL" > fit_${TAG}_train_cyl.log 2>&1 &
P1=$!; sleep 60
timeout -s INT "${TRAIN_MIN:-40}m" ./run_rl.sh sidepick45box 256 500 "$CK_BOX" > fit_${TAG}_train_box.log 2>&1 &
P2=$!; wait $P1 $P2; cleanup
for f in cyl box; do echo "[$(date '+%T')] 학습 $f 끝: $(grep -a 'Learning iteration' fit_${TAG}_train_$f.log | tail -1 | tr -s ' ') | 잡는 오차 mm $(grep -a 'grasp_fit_mm' fit_${TAG}_train_$f.log | tail -1 | tr -s ' ') | 쥐기 전 밀림 $(grep -a 'pre_push_mm' fit_${TAG}_train_$f.log | tail -1 | tr -s ' ') | 닫는 동안 속도 $(grep -a 'close_speed_cms' fit_${TAG}_train_$f.log | tail -1 | tr -s ' ')" | tee -a "$SUM"; done
timeout -s INT 8m ./run_rl.sh sidepick45_diag 64 > fit_${TAG}_diag_cyl.log 2>&1; cleanup; show fit_${TAG}_diag_cyl.log
timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 > fit_${TAG}_diag_box.log 2>&1; cleanup; show fit_${TAG}_diag_box.log
cp -f policies/sidepick45_latest.pt policies/sidepick45_$TAG.pt; cp -f policies/sidepick45box_latest.pt policies/sidepick45box_$TAG.pt
echo "[$(date '+%T')] 원기둥 상자당 2 개 (가정)" | tee -a "$SUM"
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45_$TAG.pt" > fit_${TAG}_chain_cyl.log 2>&1; cleanup; show fit_${TAG}_chain_cyl.log
echo "[$(date '+%T')] 직육면체 상자당 2 개 (가정)" | tee -a "$SUM"
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45box_$TAG.pt" > fit_${TAG}_chain_box.log 2>&1; cleanup; show fit_${TAG}_chain_box.log
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
