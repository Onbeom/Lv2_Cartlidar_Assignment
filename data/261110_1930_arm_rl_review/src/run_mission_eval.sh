#!/usr/bin/env bash
# 2026-11-09 전체 연결 평가 (단계별, 창 없이): 집기 -> 스토리지 넣기 (chain45) 와 스토리지 꺼내기 -> 놓기 (unload_place_play) 는 서로 다른 환경이라
#   한 에피소드로 잇지 않고 단계마다 평가해 요약 (mission_summary.log). 미션 한 번 = 퍽마다 '집어 넣기' x '꺼내 놓기' 성공률을 곱해 어림.
#   ./run_mission_eval.sh [집기 정책 .pt]     (기본 policies/sidepick45chain_latest.pt, 없으면 sidepick45_n5.pt) 약 40 분
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; cd "$HERE"
SP="${1:-$HERE/policies/sidepick45chain_latest.pt}"; [ -f "$SP" ] || SP="$HERE/policies/sidepick45_n5.pt"
SUM="$HERE/mission_summary.log"; N=48
cleanup() { pkill -9 -f "[c]hain45_play.py"; pkill -9 -f "[u]nload_place_play.py"; sleep 3; }
rate() { grep -aoE "$2" "$1" | tail -1 | grep -oE "[0-9]+/[0-9]+" | head -1; }
echo "===== $(date '+%F %T') 전체 연결 평가 (집기 정책 $(basename "$SP")) =====" | tee -a "$SUM"
declare -A R
run() {  # $1 이름, $2 로그, 나머지 = 명령
  local nm=$1 lg=$2; shift 2
  timeout -k 30s -s INT 12m "$@" > "$lg" 2>&1; cleanup
  grep -aqE "Traceback" "$lg" && echo "    $nm: 에러 ($lg)" | tee -a "$SUM"
}
run "세운 원기둥 넣기" ms_cyl.log ./run_rl.sh chain45_eval $N --occ 1.0 --box_prob 0 --est --perc_bias 1 --seconds 40 --sidepick "$SP"
run "세운 직육면체 넣기" ms_box.log ./run_rl.sh chain45_eval $N --occ 1.0 --box_prob 1 --est --perc_bias 1 --seconds 40 --sidepick "$SP"
run "누운 직육면체 가로 넣기" ms_lie_side.log ./run_rl.sh chain45_eval 24 --lie_regrasp --lie_yaw side --est --seconds 60
run "누운 직육면체 세로 넣기" ms_lie_radial.log ./run_rl.sh chain45_eval 24 --lie_regrasp --lie_yaw radial --est --seconds 60
run "꺼내 놓기 상자 60°" ms_unload_box.log ./run_place.sh unload $N
run "꺼내 놓기 L2 45°" ms_unload_l2.log env L2=1 ./run_place.sh unload $N
for x in "세운 원기둥 넣기:ms_cyl.log:\*\*성공[^*]*\*\*" "세운 직육면체 넣기:ms_box.log:\*\*성공[^*]*\*\*" "누운 직육면체 가로 넣기:ms_lie_side.log:\*\*성공[^*]*\*\*" \
         "누운 직육면체 세로 넣기:ms_lie_radial.log:\*\*성공[^*]*\*\*" "꺼내 놓기 상자 60°:ms_unload_box.log:꺼내 놓기 성공 [0-9]+/[0-9]+" "꺼내 놓기 L2 45°:ms_unload_l2.log:꺼내 놓기 성공 [0-9]+/[0-9]+"; do
  nm=${x%%:*}; rest=${x#*:}; lg=${rest%%:*}; pat=${rest#*:}
  v=$(rate "$lg" "$pat"); R[$nm]=${v:-0/1}
  printf "    %-22s %s\n" "$nm" "${v:-없음}" | tee -a "$SUM"
done
python3 - "${R[세운 원기둥 넣기]}" "${R[세운 직육면체 넣기]}" "${R[누운 직육면체 가로 넣기]}" "${R[누운 직육면체 세로 넣기]}" "${R[꺼내 놓기 상자 60°]}" "${R[꺼내 놓기 L2 45°]}" <<'EOF' | tee -a "$SUM"
import sys
f = [eval(a.replace("/", "/1.0/")) if "/" in a else 0.0 for a in sys.argv[1:]]
put = (f[0] + f[1]) / 2 * 0.67 + (f[2] + f[3]) / 2 * 0.33  # 세운 퍽 2/3, 누운 직육면체 1/3 가정
for nm, pl in (("상자 (L1 · L3)", f[4]), ("L2", f[5])):
    print(f"    퍽 하나 어림 ({nm}): 집어 넣기 {put * 100:.0f} % x 꺼내 놓기 {pl * 100:.0f} % = {put * pl * 100:.0f} %")
EOF
echo "===== $(date '+%F %T') 끝 =====" | tee -a "$SUM"
