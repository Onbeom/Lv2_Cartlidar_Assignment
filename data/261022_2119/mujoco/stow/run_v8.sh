#!/usr/bin/env bash
# V8 속도를 높인 모퉁이 호 회전(검증계획 23장, 모의)을 두 엔진에서 차례로 실행하고 판정한다(PC).
#   통로 {A, B} × 왼쪽 × {v 0.10: R 0.10·0.20·0.35, v 0.15: R 0.35}. 엔진마다 8회
#   MuJoCo 결과: mujoco/drive/results/v8_<YYMMDD_HHMM>/<이름>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/V8_v12_<이름>_<시각>/, 판정 요약: phys_tests/logs/v8_<YYMMDD_HHMM>.txt
# 사용: bash 261022_2119/mujoco/stow/run_v8.sh [--mujoco-only|--isaac-only]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261022_2119
cd "$REV/.."
ONLY="${1:-}"
STAMP=$(date +%y%m%d_%H%M)
MJ_PY="$REV/mujoco/.venv/bin/python"
MJ_OUT="$REV/mujoco/drive/results/v8_$STAMP"
SUM="$REV/phys_tests/logs/v8_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"

CASES=()   # "시험 이름표 인자..."
for K in A B; do
    for R in 0.10 0.20 0.35; do CASES+=("V8 ${K}L_r${R#0.}_v10 --corner $K --side L --radius-m $R --speed 0.10"); done
    CASES+=("V8 ${K}L_r35_v15 --corner $K --side L --radius-m 0.35 --speed 0.15")
done

judge() { "$MJ_PY" "$HERE/analyze_stow.py" "$1" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"; }

if [ "$ONLY" != "--isaac-only" ]; then
    echo "=== MuJoCo (dt 0.002, 바퀴 armature 0.0163, 팔 0.045)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; T=$1; NAME=$2; shift 2
        "$MJ_PY" "$REV/mujoco/drive/mujoco_drive.py" "$T" --dt 0.002 --wheel-armature 0.0163 --arm-armature 0.045 \
            "$@" --video --out "$MJ_OUT/${T}_$NAME" 2>&1 | grep -v "WARNING: Geom with duplicate name"
        judge "$MJ_OUT/${T}_$NAME"
    done
fi
if [ "$ONLY" != "--mujoco-only" ]; then
    echo "=== Isaac Sim (v12: 마찰 0.3/0.225, armature 0.0163/0.045, 팔 kp 400·kd 36)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; T=$1; NAME=$2; shift 2
        LOG="$REV/phys_tests/logs/${T}_${NAME}_$(date +%y%m%d_%H%M%S).log"
        ~/isaacsim-venv/bin/python "$REV/phys_tests/run_phys_test.py" --test "$T" "$@" \
            --wheel-armature 0.0163 --arm-armature 0.045 --arm-kd 36 --tag "v12_$NAME" \
            --usd "$REV/phys_tests/usd_copy/waffle_4wd/waffle_4wd.usda" \
            --urdf "$REV/urdf/waffle_4wd_description_261022_2119/urdf/waffle_4wd.urdf" \
            --out "$REV/phys_tests/results" --/crashreporter/url= > "$LOG" 2>&1
        R=$(grep -oE "RESULT_DIR: .+" "$LOG" | tail -1 | cut -d' ' -f2)
        if [ -n "$R" ]; then judge "$R"; else echo "Isaac $T $NAME: 결과 폴더 없음(로그 $LOG 확인)" | tee -a "$SUM"; fi
    done
fi
echo "MuJoCo 결과: $MJ_OUT"
echo "판정 요약: $SUM"
