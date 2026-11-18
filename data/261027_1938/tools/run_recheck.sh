#!/usr/bin/env bash
# 261027_1711 물리 동작 재검사(검증계획_261027_1711.md, 모의, PC). 261027_1530 때 돌린 시험에 대체된 시험(V1b · V6 · V6b · V7)까지 모두 돌린다.
#   바퀴 armature는 l2/config/wheels.yaml 값. MuJoCo는 CPU라 잠금 없이, Isaac은 GPU 잠금 + 메모리 제한 아래.
# 사용: bash 261027_1938/tools/run_recheck.sh [시작 단계 1~12]
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
both() {  # 두 엔진 V 시험: MuJoCo 먼저(잠금 없음), Isaac은 잠금 아래. 둘째 인자부터는 실행기에 그대로
    local s=$1; shift
    bash "$W/mujoco/stow/$s" --mujoco-only "$@" 2>&1 | tee -a "$SUM" | grep -E "판정 요약|MuJoCo 결과" || true
    gpu bash "$W/mujoco/stow/$s" --isaac-only "$@" 2>&1 | tee -a "$SUM" | grep -E "판정 요약" || true
}
echo "바퀴 armature $WA, URDF $URDF" | tee -a "$SUM"
[ "$START" -le 1 ] && { step "1 MuJoCo M 묶음"; bash "$W/mujoco/drive/run_all.sh" 2>&1 | grep -E "판정|결과 폴더" | tee -a "$SUM"; }
if [ "$START" -le 2 ]; then step "2 Isaac P1~P8"
    gpu env EXTRA_ARGS="--wheel-armature $WA --arm-armature $AA --arm-kd 36 --tag v12" \
        bash "$W/phys_tests/run_all.sh" --headless P1 P2 P3 P4on P4off P5 P6 P7 2>&1 | tee -a "$SUM" | grep -E "시작|판정" || true
fi
[ "$START" -le 3 ] && { step "3 V1~V3"; both run_v1v3.sh; }
if [ "$START" -le 4 ]; then step "4 V1b(오프셋 = 이번 V1 회전 중심 x: MuJoCo · Isaac · 평균)"
    OFF=$(python3 - "$W" <<'EOF'
import glob, json, sys
w = sys.argv[1]
mj = sorted(glob.glob(f"{w}/mujoco/drive/results/v1v3_*/V1_stow/verdict_stow.json"))[-1]
isc = sorted(glob.glob(f"{w}/phys_tests/results/V1_v12_stow_*/verdict_stow.json"))[-1]
a, b = (abs(json.load(open(f))["measured"]["pivot_mm"][0]) / 1000 for f in (mj, isc))
print(f"{a:.4f} {(a + b) / 2:.4f} {b:.4f}")
EOF
)
    echo "V1b 오프셋 $OFF m" | tee -a "$SUM"
    gpu env V1B_OFFSETS="$OFF" bash "$W/mujoco/stow/run_v1b.sh" 2>&1 | tee -a "$SUM" | grep -E "판정 요약" || true
fi
[ "$START" -le 5 ] && { step "5 V4·V5"; both run_v4v5.sh; }
[ "$START" -le 6 ] && { step "6 V6·V6b"; both run_v6.sh --v6b; }
[ "$START" -le 7 ] && { step "7 V7"; both run_v7.sh; }
[ "$START" -le 8 ] && { step "8 V6c·V6cb·V7c"; both run_v6c.sh; }
[ "$START" -le 9 ] && { step "9 V8"; both run_v8.sh; }
[ "$START" -le 10 ] && { step "10 V9"; both run_v9.sh; }
if [ "$START" -le 11 ]; then step "11 L2 600 s"
    gpu bash "$W/l2/run_l2.sh" 600 "$W/results" 2>&1 | tee -a "$SUM" | tail -5
fi
step "끝. 요약 $SUM"
