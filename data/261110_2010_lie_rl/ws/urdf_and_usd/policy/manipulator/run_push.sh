#!/usr/bin/env bash
# 2026-11-06 퍽 자리 무작위 (정지 거리 그대로) + 쥐기 전 밀기 · 닫는 동안 팔 멈춤 벌점:
#   1) 학습 전 기준 = fit1 정책을 새 배치에서 (진단 64 x 2 + 상자당 2 개 48 x 2)  2) fit1 에서 500 반복 이어 학습 (run_fit.sh, TAG=push1) -> 진단 -> 상자당 2 개
#   ./run_push.sh   (약 1 시간 10 분, 요약 fit_summary.log)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SUM="$HERE/fit_summary.log"
KEYS='한 번 시도 기준|팔-몸체 \(1 N|다 채운 대|끝 분류|손 안 퍽 가정 오차 \|mm\||팔끼리 충돌'
cleanup() { pkill -f "[s]idepick_diag.py" 2>/dev/null; pkill -f "[c]hain45_play.py" 2>/dev/null; sleep 5; }
show() { grep -aE "$KEYS" "$1" | sed 's/대마다 연속으로 앉힌 수 \[[^]]*\], //' | sed "s/^/    /" | cut -c1-230 | tee -a "$SUM"; }
echo "===== $(date '+%F %T') push1 학습 전 기준 (fit1, 퍽 자리 무작위 배치) =====" | tee -a "$SUM"
timeout -s INT 8m ./run_rl.sh sidepick45_diag 64 "$HERE/policies/sidepick45_fit1.pt" > push_base_diag_cyl.log 2>&1; cleanup; show push_base_diag_cyl.log
grep -aq "Traceback" push_base_diag_cyl.log && { echo "에러: push_base_diag_cyl.log" | tee -a "$SUM"; exit 1; }
timeout -s INT 8m ./run_rl.sh sidepick45box_diag 64 "$HERE/policies/sidepick45box_fit1.pt" > push_base_diag_box.log 2>&1; cleanup; show push_base_diag_box.log
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45_fit1.pt" > push_base_chain_cyl.log 2>&1; cleanup; show push_base_chain_cyl.log
timeout -s INT 10m ./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --box_prob 1.0 --est --seconds 40 --sidepick "$HERE/policies/sidepick45box_fit1.pt" > push_base_chain_box.log 2>&1; cleanup; show push_base_chain_box.log
TAG=push1 CK_CYL="$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-06_13-10-50/model_6348.pt" CK_BOX="$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-06_13-11-51/model_28790.pt" ./run_fit.sh
