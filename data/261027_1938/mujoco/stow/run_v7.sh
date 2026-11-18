#!/usr/bin/env bash
# V7 위치 추정 오차가 있을 때의 정상화(검증계획 21장, 모의)를 두 엔진에서 차례로 실행하고 판정한다(PC).
#   시작 방향 {+90, 180}° × 오차 {E1 편향, E2 잡음 seed 1·2, E3 편향 + 잡음}. 엔진마다 8회
#   MuJoCo 결과: mujoco/drive/results/v7_<YYMMDD_HHMM>/<이름>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/V7_v12_<이름>_<시각>/, 판정 요약: phys_tests/logs/v7_<YYMMDD_HHMM>.txt
# 사용: bash 261027_1938/mujoco/stow/run_v7.sh [--mujoco-only|--isaac-only]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261027_1938
source "$REV/conditions.sh"
cd "$REV/.."
ONLY="${1:-}"
STAMP=$(date +%y%m%d_%H%M)
MJ_OUT="$REV/mujoco/drive/results/v7_$STAMP"
SUM="$REV/phys_tests/logs/v7_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"

CASES=()   # "시험 이름표 인자..."
for H in 90 180; do
    CASES+=("V7 h${H}_E1 --start-heading-deg $H --loc-bias 0.005 0.5")
    CASES+=("V7 h${H}_E2s1 --start-heading-deg $H --loc-noise 0.010 1.0 --seed 1")
    CASES+=("V7 h${H}_E2s2 --start-heading-deg $H --loc-noise 0.010 1.0 --seed 2")
    CASES+=("V7 h${H}_E3 --start-heading-deg $H --loc-bias 0.005 0.5 --loc-noise 0.010 1.0 --seed 1")
done

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
