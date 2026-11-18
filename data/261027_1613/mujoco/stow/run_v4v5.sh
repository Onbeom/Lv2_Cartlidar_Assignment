#!/usr/bin/env bash
# V4 모퉁이 호 회전·V5 막다른 길 후진(검증계획 19장, 모의)을 두 엔진에서 차례로 실행하고 판정한다(PC).
#   V4: 통로 {A, B} × 방향 {L, R} × 반경 {0.10, 0.20, 0.35} m, V5: 통로 {A, B}. 엔진마다 14회
#   MuJoCo 결과: mujoco/drive/results/v4v5_<YYMMDD_HHMM>/<이름>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/V4_v12_<이름>_<시각>/ 등, 판정 요약: phys_tests/logs/v4v5_<YYMMDD_HHMM>.txt
# 사용: bash 261027_1613/mujoco/stow/run_v4v5.sh [--mujoco-only|--isaac-only]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261027_1613
source "$REV/conditions.sh"
cd "$REV/.."
ONLY="${1:-}"
STAMP=$(date +%y%m%d_%H%M)
MJ_OUT="$REV/mujoco/drive/results/v4v5_$STAMP"
SUM="$REV/phys_tests/logs/v4v5_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"

CASES=()   # "시험 이름표 인자..."
for K in A B; do for S in L R; do for R in 0.10 0.20 0.35; do
    CASES+=("V4 ${K}${S}_r${R#0.} --corner $K --side $S --radius-m $R")
done; done; done
for K in A B; do CASES+=("V5 dead${K} --deadend $K"); done

judge() { "$MJ_PY" "$HERE/analyze_stow.py" "$1" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"; }

if [ "$ONLY" != "--isaac-only" ]; then
    echo "=== MuJoCo (dt 0.002, 바퀴 armature $WA, 팔 0.045)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; T=$1; NAME=$2; shift 2
        "$MJ_PY" "$REV/mujoco/drive/mujoco_drive.py" "$T" --dt 0.002 --wheel-armature "$WA" --arm-armature 0.045 \
            "$@" --video --out "$MJ_OUT/${T}_$NAME" 2>&1 | grep -v "WARNING: Geom with duplicate name"
        judge "$MJ_OUT/${T}_$NAME"
    done
fi
if [ "$ONLY" != "--mujoco-only" ]; then
    echo "=== Isaac Sim (v12: 마찰 0.3/0.225, armature $WA/0.045, 팔 kp 400·kd 36)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; T=$1; NAME=$2; shift 2
        LOG="$REV/phys_tests/logs/${T}_${NAME}_$(date +%y%m%d_%H%M%S).log"
        ~/isaacsim-venv/bin/python "$REV/phys_tests/run_phys_test.py" --test "$T" "$@" \
            --wheel-armature "$WA" --arm-armature 0.045 --arm-kd 36 --tag "v12_$NAME" \
            --usd "$USD" \
            --urdf "$URDF" \
            --out "$REV/phys_tests/results" --/crashreporter/url= > "$LOG" 2>&1
        R=$(grep -oE "RESULT_DIR: .+" "$LOG" | tail -1 | cut -d' ' -f2)
        if [ -n "$R" ]; then judge "$R"; else echo "Isaac $T $NAME: 결과 폴더 없음(로그 $LOG 확인)" | tee -a "$SUM"; fi
    done
fi
echo "MuJoCo 결과: $MJ_OUT"
echo "판정 요약: $SUM"
