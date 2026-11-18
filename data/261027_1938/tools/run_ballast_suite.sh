#!/usr/bin/env bash
# 앞 무게추 0.757 kg(추천 자리) 가정 시험(모의, PC). 261027_1711 + 무게추 사본(261027_1912/variants/final_g0757).
#   1 USD 가져오기  2 최고 속도(M5 · P5) · 직진 정지(M6 · P6)  3 V1~V3  4 V4 · V5  5 V6c · V6cb · V7c
#   6 규칙 최고 속도 모퉁이(V8: R 0.20, v 0.205 m/s, 통로 A · B × 좌우)  7 마찰 0.8(V4 A-L R 0.20, V6 +90° · 180°)
# 사용: bash 261027_1938/tools/run_ballast_suite.sh [시작 단계]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W="$(dirname "$HERE")"
cd "$W/.."
source "$W/conditions.sh"
V=/home/pa4/git/lv2_next_module/261027_1912/variants/final_g0757
START=${1:-1}
SUM="$W/suite_$(date +%y%m%d_%H%M).txt"
MJ_OUT="$W/mujoco/drive/results/suite"
IS_OUT="$W/phys_tests/results"
mkdir -p "$MJ_OUT" "$IS_OUT" "$W/phys_tests/logs"
gpu() { flock /tmp/lv2_gpu.lock systemd-run --user -P --wait --collect -q -p MemoryMax=12G -p MemorySwapMax=0 --working-directory="$PWD" "$@"; }
step() { echo "=== [$(date +%H:%M:%S)] $*" | tee -a "$SUM"; }
judge() { "$MJ_PY" "$W/mujoco/stow/analyze_stow.py" "$1" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"; }
mj() {  # mj <시험> <이름> [인자...]
    local T=$1 NAME=$2; shift 2
    "$MJ_PY" "$W/mujoco/drive/mujoco_drive.py" "$T" --dt 0.002 --wheel-armature "$WA" --arm-armature "$AA" "$@" --video \
        --out "$MJ_OUT/${T}_$NAME" 2>&1 | grep -v "duplicate name" | tail -1
}
is() {  # is <시험> <이름> [인자...] → 결과 폴더 경로 출력
    local T=$1 NAME=$2; shift 2
    local LOG="$W/phys_tests/logs/${T}_${NAME}_$(date +%y%m%d_%H%M%S).log"
    gpu ~/isaacsim-venv/bin/python "$W/phys_tests/run_phys_test.py" --test "$T" "$@" --wheel-armature "$WA" --arm-armature "$AA" \
        --arm-kd 36 --tag "v12_$NAME" --usd "$USD" --urdf "$URDF" --out "$IS_OUT" --/crashreporter/url= > "$LOG" 2>&1
    grep -oE "RESULT_DIR: .+" "$LOG" | tail -1 | cut -d' ' -f2
}
if [ "$START" -le 1 ]; then step "1 USD 가져오기"
    [ -f "$USD" ] || gpu ~/isaacsim-venv/bin/python sim/isaac/import_waffle_4wd.py --urdf "$URDF" --package-dir "$V/waffle_4wd_description_261027_1711" \
        --out "$V/usd" --/crashreporter/url= > "$V/import_final.log" 2>&1
    ls "$USD" | tee -a "$SUM"
fi
if [ "$START" -le 2 ]; then step "2 최고 속도 · 직진 정지"
    for T in M5 M6; do mj $T ballast; python3 "$W/mujoco/drive/analyze_drive.py" "$MJ_OUT/${T}_ballast" > /dev/null 2>&1; done
    for T in P5 P6; do R=$(is $T ballast); echo "Isaac $T → $R" | tee -a "$SUM"; python3 "$W/phys_tests/analyze_phys_test.py" "$R" > /dev/null 2>&1; done
fi
[ "$START" -le 3 ] && { step "3 V1~V3"; bash "$W/mujoco/stow/run_v1v3.sh" --mujoco-only 2>&1 | tee -a "$SUM" | tail -1; gpu bash "$W/mujoco/stow/run_v1v3.sh" --isaac-only 2>&1 | tee -a "$SUM" | tail -1; }
[ "$START" -le 4 ] && { step "4 V4 · V5"; bash "$W/mujoco/stow/run_v4v5.sh" --mujoco-only 2>&1 | tee -a "$SUM" | tail -1; gpu bash "$W/mujoco/stow/run_v4v5.sh" --isaac-only 2>&1 | tee -a "$SUM" | tail -1; }
[ "$START" -le 5 ] && { step "5 V6c · V6cb · V7c"; bash "$W/mujoco/stow/run_v6c.sh" --mujoco-only 2>&1 | tee -a "$SUM" | tail -1; gpu bash "$W/mujoco/stow/run_v6c.sh" --isaac-only 2>&1 | tee -a "$SUM" | tail -1; }
if [ "$START" -le 6 ]; then step "6 규칙 최고 속도 모퉁이 V8 R 0.20 · v 0.205"
    for K in A B; do for S in L R; do
        mj V8 "${K}${S}_r20_v205" --corner $K --side $S --radius-m 0.20 --speed 0.205; judge "$MJ_OUT/V8_${K}${S}_r20_v205"
        R=$(is V8 "${K}${S}_r20_v205" --corner $K --side $S --radius-m 0.20 --speed 0.205); judge "$R"
    done; done
fi
if [ "$START" -le 7 ]; then step "7 마찰 0.8"
    for C in "V4 AL_r20_mu08 --corner A --side L --radius-m 0.20" "V6 h90_mu08 --start-heading-deg 90" "V6 h180_mu08 --start-heading-deg 180"; do
        set -- $C; T=$1; NAME=$2; shift 2
        mj $T "$NAME" "$@" --mu 0.8; judge "$MJ_OUT/${T}_$NAME"
        R=$(is $T "$NAME" "$@" --mu 0.8 0.6); judge "$R"
    done
fi
step "끝. 요약 $SUM"
