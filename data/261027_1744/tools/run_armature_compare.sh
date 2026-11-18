#!/usr/bin/env bash
# 바퀴 armature 비교 실험(검증계획_261027_1530.md 6장, 모의, PC). 후보 A 0.007244 · B 0.0163.
#   MuJoCo: M 묶음(dt A·B·C, A만 민감도 포함) + V1. Isaac: P5·P6·P7·V1 (GPU 잠금 + 메모리 제한 아래)
# 사용: bash 261027_1744/tools/run_armature_compare.sh
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W="$(dirname "$HERE")"
cd "$W/.."
OUT="$W/arm_cmp"
mkdir -p "$OUT"
SUM="$OUT/summary_$(date +%y%m%d_%H%M).txt"
gpu() { flock /tmp/lv2_gpu.lock systemd-run --user -P --wait --collect -q -p MemoryMax=12G -p MemorySwapMax=0 --working-directory="$PWD" "$@"; }
for WA in 0.007244 0.0163; do
    export WA
    source "$W/conditions.sh"
    echo "=== 후보 WA=$WA" | tee -a "$SUM"
    if [ "$WA" = "0.007244" ]; then SK=""; else SK=1; fi
    OUT="$OUT/M_wa$WA" SKIP_SENS=$SK bash "$W/mujoco/drive/run_all.sh" 2>&1 | grep -E "판정|결과 폴더|Error|Traceback" | tee -a "$SUM"
    "$MJ_PY" "$W/mujoco/drive/mujoco_drive.py" V1 --dt 0.002 --wheel-armature "$WA" --arm-armature "$AA" --video \
        --out "$OUT/V1_mj_wa$WA" 2>&1 | grep -v "duplicate name"
    "$MJ_PY" "$W/mujoco/stow/analyze_stow.py" "$OUT/V1_mj_wa$WA" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"
    for T in P5 P6 P7 V1; do
        LOG="$OUT/${T}_isaac_wa$WA.log"
        gpu ~/isaacsim-venv/bin/python "$W/phys_tests/run_phys_test.py" --test "$T" \
            --wheel-armature "$WA" --arm-armature "$AA" --arm-kd 36 --tag "v12_wa$WA" \
            --usd "$USD" --urdf "$URDF" --out "$OUT/isaac" --/crashreporter/url= > "$LOG" 2>&1
        R=$(grep -oE "RESULT_DIR: .+" "$LOG" | tail -1 | cut -d' ' -f2)
        if [ -z "$R" ]; then echo "Isaac $T WA=$WA: 결과 폴더 없음($LOG)" | tee -a "$SUM"; continue; fi
        if [ "$T" = "V1" ]; then "$MJ_PY" "$W/mujoco/stow/analyze_stow.py" "$R" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"
        else python3 "$W/phys_tests/analyze_phys_test.py" "$R" > /dev/null 2>&1; echo "Isaac $T WA=$WA → $R" | tee -a "$SUM"; fi
    done
    unset WA
done
echo "끝. 요약 $SUM" | tee -a "$SUM"
