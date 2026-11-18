#!/usr/bin/env bash
# 261027_1530 물리 동작 재검사 본 시험(검증계획_261027_1530.md 5장 순서, 모의, PC).
#   바퀴 armature는 l2/config/wheels.yaml 값(6장 비교로 고른 값). MuJoCo는 CPU라 잠금 없이, Isaac은 GPU 잠금 + 메모리 제한 아래.
# 사용: bash 261027_1613/tools/run_recheck.sh [시작 단계 1~8]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W="$(dirname "$HERE")"
cd "$W/.."
unset WA
source "$W/conditions.sh"
START=${1:-1}
SUM="$W/recheck_$(date +%y%m%d_%H%M).txt"
gpu() { flock /tmp/lv2_gpu.lock systemd-run --user -P --wait --collect -q -p MemoryMax=12G -p MemorySwapMax=0 --working-directory="$PWD" "$@"; }
step() { echo "=== [$(date +%H:%M:%S)] $*" | tee -a "$SUM"; }
both() {  # 두 엔진 V 시험: MuJoCo 먼저(잠금 없음), Isaac은 잠금 아래
    bash "$W/mujoco/stow/$1" --mujoco-only 2>&1 | tee -a "$SUM" | grep -E "판정 요약|MuJoCo 결과" || true
    gpu bash "$W/mujoco/stow/$1" --isaac-only 2>&1 | tee -a "$SUM" | grep -E "판정 요약" || true
}
echo "바퀴 armature $WA" | tee -a "$SUM"
if [ "$START" -le 1 ]; then step "1 MuJoCo M 묶음"
    if [ -d "$W/arm_cmp/M_wa$WA" ] && ls "$W/arm_cmp/M_wa$WA" | grep -q "_wa0$"; then
        echo "armature 비교 때 같은 값으로 민감도까지 돌린 결과를 씀: $W/arm_cmp/M_wa$WA" | tee -a "$SUM"
    else bash "$W/mujoco/drive/run_all.sh" 2>&1 | grep -E "판정|결과 폴더" | tee -a "$SUM"; fi
fi
if [ "$START" -le 2 ]; then step "2 Isaac P1~P8"
    gpu env EXTRA_ARGS="--wheel-armature $WA --arm-armature $AA --arm-kd 36 --tag v12" \
        bash "$W/phys_tests/run_all.sh" --headless P1 P2 P3 P4on P4off P5 P6 P7 2>&1 | tee -a "$SUM" | grep -E "시작|판정" || true
fi
[ "$START" -le 3 ] && { step "3 V1~V3"; both run_v1v3.sh; }
[ "$START" -le 4 ] && { step "4 V4·V5"; both run_v4v5.sh; }
[ "$START" -le 5 ] && { step "5 V6c·V6cb·V7c"; both run_v6c.sh; }
[ "$START" -le 6 ] && { step "6 V8"; both run_v8.sh; }
[ "$START" -le 7 ] && { step "7 V9"; both run_v9.sh; }
if [ "$START" -le 8 ]; then step "8 L2 600 s"
    gpu bash "$W/l2/run_l2.sh" 600 "$W/results" 2>&1 | tee -a "$SUM" | tail -5
fi
step "끝. 요약 $SUM"
