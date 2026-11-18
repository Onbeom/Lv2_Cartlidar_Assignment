#!/usr/bin/env bash
# 2026-11-05 주행 정지 기준 배치 (상자 앞면 0.17 m) + 상자당 2 개: 학습 전 기준 -> 이어 학습 (원기둥 · 직육면체 동시) -> 진단 -> 상자당 2 개 평가를 한 번에.
# 단계마다 로그는 dock2_<단계>.log, 요약은 dock2_summary.log. 1 단계에서 에러가 나면 학습 전에 멈춤.
#
#   ./run_dock2.sh            (컨테이너 안에서, 약 1 시간)
#   TRAIN_MIN=20 ./run_dock2.sh   (학습을 20 분에서 끊음, 체크포인트는 50 반복마다 저장)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
SUM="$HERE/dock2_summary.log"
CK_CYL="$HERE/logs/rsl_rl/waffle_sidepick45/2026-11-04_16-00-30/model_4940.pt"      # SP-45c
CK_BOX="$HERE/logs/rsl_rl/waffle_sidepick45box/2026-11-05_06-16-14/model_23403.pt"  # 밤샘 35 회차
KEYS='한 번 시도|팔-몸체 \(1 N|든 퍽 기울기 최대|연속|끝 분류|\*\*성공|칸별|앞 칸 퍽 닿은|두 번째 퍽'

say() { echo "[$(date '+%T')] $*" | tee -a "$SUM"; }
cleanup() {
  pkill -f "[t]rain.py --task Waffle-SidePick45" 2>/dev/null
  pkill -f "[s]idepick_diag.py" 2>/dev/null
  pkill -f "[c]hain45_play.py" 2>/dev/null
  sleep 5
}
run() {  # run <로그 이름> <제한 분> <명령...>
  local name=$1 lim=$2; shift 2
  say "$name 시작: $*"
  timeout -s INT "${lim}m" "$@" > "dock2_$name.log" 2>&1; cleanup
  grep -aE "$KEYS" "dock2_$name.log" | grep -av "단계 \[" | sed "s/^/    /" | tee -a "$SUM"
  if grep -aqE "Traceback|Error executing|AssertionError" "dock2_$name.log"; then say "$name 에러 (dock2_$name.log)"; return 1; fi
}
chain() {  # chain <로그 이름> [추가 인자...] : 상자당 2 개, 칸 채우는 순서 왼 -> 오, 앞 칸 다 참, 실물 조건 (est), 24 대 = 짝 칸 6 가지 x 2
  local name=$1; shift
  run "$name" 15 ./run_rl.sh chain45_eval 24 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 "$@"
}

echo "===== $(date '+%F %T') 시작 =====" | tee -a "$SUM"
# 1 학습 전 기준 (지금 정책, 새 배치): 코드가 도는지 확인 겸
run base_diag_cyl 8 ./run_rl.sh sidepick45_diag 64 "$HERE/policies/sidepick45_SP45c_4940.pt" || exit 1
run base_diag_box 8 ./run_rl.sh sidepick45box_diag 64 "$HERE/policies/sidepick45box_r35.pt" || exit 1
chain base_chain_cyl --sidepick "$HERE/policies/sidepick45_SP45c_4940.pt" || exit 1
chain base_chain_box --box_prob 1.0 --sidepick "$HERE/policies/sidepick45box_r35.pt" || exit 1

# 2 이어 학습 500 반복, 원기둥 (혼합) · 직육면체 동시
say "학습 시작 (원기둥 $CK_CYL, 직육면체 $CK_BOX)"
timeout -s INT "${TRAIN_MIN:-60}m" ./run_rl.sh sidepick45 256 500 "$CK_CYL" > dock2_train_cyl.log 2>&1 &
P1=$!
sleep 60  # 두 프로세스가 동시에 GPU 를 잡지 않게
timeout -s INT "${TRAIN_MIN:-60}m" ./run_rl.sh sidepick45box 256 500 "$CK_BOX" > dock2_train_box.log 2>&1 &
P2=$!
wait $P1 $P2; cleanup
for f in cyl box; do say "학습 $f 끝: $(grep -a 'Learning iteration' dock2_train_$f.log | tail -1 | tr -s ' ')"; done

# 3 진단 (새 정책을 policies/sidepick45_latest.pt · sidepick45box_latest.pt 로 내보냄)
run diag_cyl 8 ./run_rl.sh sidepick45_diag 64
run diag_box 8 ./run_rl.sh sidepick45box_diag 64

# 4 상자당 2 개 평가 (새 정책)
chain chain_cyl --sidepick "$HERE/policies/sidepick45_latest.pt"
chain chain_box --box_prob 1.0 --sidepick "$HERE/policies/sidepick45box_latest.pt"
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
