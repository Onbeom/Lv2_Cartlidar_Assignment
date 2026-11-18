#!/usr/bin/env bash
# V1b 회전 중심 오프셋 보정 회전(검증계획 18장, 모의)을 두 엔진에서 차례로 실행하고 판정한다(PC).
#   오프셋 d ∈ {52.0, 65.9, 79.7} mm × MuJoCo(dt 0.002, 기준 조건)·Isaac v12(dt 1/120) = 6회
#   MuJoCo 결과: mujoco/drive/results/v1b_<YYMMDD_HHMM>/V1b_d<mm>/ (video.mp4 포함)
#   Isaac 결과: phys_tests/results/V1b_v12_d<mm>_<시각>/, 판정 요약: phys_tests/logs/v1b_v12_<YYMMDD_HHMM>.txt
# 사용: bash 261027_1613/mujoco/stow/run_v1b.sh [--gui]   (--gui: Isaac 창을 띄워 실제 시간 속도로 보여 준다)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REV="$(cd "$HERE/../.." && pwd)"          # 261027_1613
source "$REV/conditions.sh"
cd "$REV/.."
GUI=""; [ "${1:-}" = "--gui" ] && GUI=--gui
STAMP=$(date +%y%m%d_%H%M)
MJ_OUT="$REV/mujoco/drive/results/v1b_$STAMP"
SUM="$REV/phys_tests/logs/v1b_v12_$STAMP.txt"
mkdir -p "$MJ_OUT" "$REV/phys_tests/logs" "$REV/phys_tests/results"
OFFSETS="0.0520 0.0659 0.0797"

echo "=== MuJoCo V1b (dt 0.002, 바퀴 armature $WA, 팔 0.045)" | tee -a "$SUM"
for D in $OFFSETS; do
    MM=$(python3 -c "print(round($D*1000))")
    "$MJ_PY" "$REV/mujoco/drive/mujoco_drive.py" V1b --dt 0.002 --wheel-armature "$WA" --arm-armature 0.045 \
        --pivot-offset-m "$D" --video --out "$MJ_OUT/V1b_d$MM" 2>&1 | grep -v "WARNING: Geom with duplicate name"
    "$MJ_PY" "$HERE/analyze_stow.py" "$MJ_OUT/V1b_d$MM" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"
done

echo "=== Isaac Sim V1b (v12: 마찰 0.3/0.225, armature $WA/0.045, 팔 kp 400·kd 36)" | tee -a "$SUM"
for D in $OFFSETS; do
    MM=$(python3 -c "print(round($D*1000))")
    LOG="$REV/phys_tests/logs/V1b_d${MM}_$(date +%y%m%d_%H%M%S).log"
    ~/isaacsim-venv/bin/python "$REV/phys_tests/run_phys_test.py" --test V1b $GUI \
        --wheel-armature "$WA" --arm-armature 0.045 --arm-kd 36 --pivot-offset-m "$D" --tag "v12_d$MM" \
        --usd "$USD" \
        --urdf "$URDF" \
        --out "$REV/phys_tests/results" --/crashreporter/url= > "$LOG" 2>&1
    R=$(grep -oE "RESULT_DIR: .+" "$LOG" | tail -1 | cut -d' ' -f2)
    if [ -n "$R" ]; then
        "$MJ_PY" "$HERE/analyze_stow.py" "$R" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"
    else
        echo "Isaac d=$MM mm: 결과 폴더 없음(로그 $LOG 확인)" | tee -a "$SUM"
    fi
done
echo "MuJoCo 결과: $MJ_OUT"
echo "판정 요약: $SUM"
