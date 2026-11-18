#!/usr/bin/env bash
# V9 바닥 마찰 민감도(검증계획 24장, 모의)를 두 엔진에서 차례로 실행하고 판정한다(PC).
#   마찰 μ {0.4, 0.6, 0.8}(부드러운 일반 카펫 추정값, Isaac 운동 마찰 = 0.75 μ) × (V4 A-L R 0.20·0.35, V6 +90°·180°). 엔진마다 12회
#   MuJoCo 결과: mujoco/drive/results/v9_<YYMMDD_HHMM>/<이름>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/<시험>_v12_<이름>_<시각>/, 판정 요약: phys_tests/logs/v9_<YYMMDD_HHMM>.txt
# 사용: bash 261027_1744/mujoco/stow/run_v9.sh [--mujoco-only|--isaac-only]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261027_1744
source "$REV/conditions.sh"
cd "$REV/.."
ONLY="${1:-}"
STAMP=$(date +%y%m%d_%H%M)
MJ_OUT="$REV/mujoco/drive/results/v9_$STAMP"
SUM="$REV/phys_tests/logs/v9_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"

CASES=()   # "μ 시험 이름표 인자..."
for MU in 0.4 0.6 0.8; do
    M=${MU/./}
    CASES+=("$MU V4 AL_r20_mu$M --corner A --side L --radius-m 0.20")
    CASES+=("$MU V4 AL_r35_mu$M --corner A --side L --radius-m 0.35")
    CASES+=("$MU V6 h90_mu$M --start-heading-deg 90")
    CASES+=("$MU V6 h180_mu$M --start-heading-deg 180")
done

judge() { "$MJ_PY" "$HERE/analyze_stow.py" "$1" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"; }

if [ "$ONLY" != "--isaac-only" ]; then
    echo "=== MuJoCo (dt 0.002, 바퀴 armature $WA, 팔 0.045)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; MU=$1; T=$2; NAME=$3; shift 3
        "$MJ_PY" "$REV/mujoco/drive/mujoco_drive.py" "$T" --dt 0.002 --wheel-armature "$WA" --arm-armature 0.045 \
            --mu "$MU" "$@" --video --out "$MJ_OUT/${T}_$NAME" 2>&1 | grep -v "WARNING: Geom with duplicate name"
        judge "$MJ_OUT/${T}_$NAME"
    done
fi
if [ "$ONLY" != "--mujoco-only" ]; then
    echo "=== Isaac Sim (v12에서 마찰만 바꿈, armature $WA/0.045, 팔 kp 400·kd 36)" | tee -a "$SUM"
    for C in "${CASES[@]}"; do
        set -- $C; MU=$1; T=$2; NAME=$3; shift 3
        MUD=$(python3 -c "print(round(0.75*$MU, 4))")
        LOG="$REV/phys_tests/logs/${T}_${NAME}_$(date +%y%m%d_%H%M%S).log"
        ~/isaacsim-venv/bin/python "$REV/phys_tests/run_phys_test.py" --test "$T" "$@" --mu "$MU" "$MUD" \
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
