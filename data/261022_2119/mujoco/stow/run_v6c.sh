#!/usr/bin/env bash
# 정지 후 헤딩 보정 확인 시험 V6c·V6cb·V7c(검증계획 22장, 모의)를 두 엔진에서 차례로 실행하고 판정한다(PC).
#   V6c: 시작 방향 {+90, 180, −90}°, V6cb: 통로 {A, B} × {L, R} × R {0.10, 0.20, 0.35},
#   V7c: 시작 방향 {+90, 180}° × 오차 {E1, E2 seed 1·2, E3}. 엔진마다 23회
#   MuJoCo 결과: mujoco/drive/results/v6c_<YYMMDD_HHMM>/<이름>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/<시험>_v12_<이름>_<시각>/, 판정 요약: phys_tests/logs/v6c_<YYMMDD_HHMM>.txt
# 사용: bash 261022_2119/mujoco/stow/run_v6c.sh [--mujoco-only|--isaac-only]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261022_2119
cd "$REV/.."
ONLY="${1:-}"
STAMP=$(date +%y%m%d_%H%M)
MJ_PY="$REV/mujoco/.venv/bin/python"
MJ_OUT="$REV/mujoco/drive/results/v6c_$STAMP"
SUM="$REV/phys_tests/logs/v6c_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"

CASES=()   # "시험 이름표 인자..."
for H in 90 180 -90; do CASES+=("V6c h${H/-/m} --start-heading-deg $H"); done
for K in A B; do for S in L R; do for R in 0.10 0.20 0.35; do
    CASES+=("V6cb ${K}${S}_r${R#0.} --corner $K --side $S --radius-m $R")
done; done; done
for H in 90 180; do
    CASES+=("V7c h${H}_E1 --start-heading-deg $H --loc-bias 0.005 0.5")
    CASES+=("V7c h${H}_E2s1 --start-heading-deg $H --loc-noise 0.010 1.0 --seed 1")
    CASES+=("V7c h${H}_E2s2 --start-heading-deg $H --loc-noise 0.010 1.0 --seed 2")
    CASES+=("V7c h${H}_E3 --start-heading-deg $H --loc-bias 0.005 0.5 --loc-noise 0.010 1.0 --seed 1")
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
