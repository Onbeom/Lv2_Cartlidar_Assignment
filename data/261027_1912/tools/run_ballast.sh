#!/usr/bin/env bash
# 앞 무게추 실험(모의): 무게추 0.5 · 1.0 · 1.5 kg 사본으로 V1(접은 채 제자리 회전)을 두 엔진에서 돌리고 판정한다(PC).
#   조건은 261027_1744 재검사와 같다(conditions.sh: armature 0.007244/0.045, Isaac v12). Isaac은 사본마다 USD를 새로 가져온다.
# 사용: bash 261027_1912/tools/run_ballast.sh
set -uo pipefail
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOP="$(dirname "$W")"
cd "$TOP"
source "$TOP/261027_1744/conditions.sh"
SUM="$W/results/summary_$(date +%y%m%d_%H%M).txt"
gpu() { flock /tmp/lv2_gpu.lock systemd-run --user -P --wait --collect -q -p MemoryMax=12G -p MemorySwapMax=0 --working-directory="$PWD" "$@"; }
judge() { "$MJ_PY" "$TOP/261027_1744/mujoco/stow/analyze_stow.py" "$1" 2>&1 | grep -v "duplicate name" | tee -a "$SUM"; }
for T in ${TAGS:-m05 m10 m15}; do
    V="$W/variants/$T"
    echo "=== $T MuJoCo" | tee -a "$SUM"
    "$MJ_PY" "$TOP/261027_1744/mujoco/drive/mujoco_drive.py" V1 --dt 0.002 --wheel-armature "$WA" --arm-armature "$AA" \
        --urdf "$V/waffle_4wd_mujoco_drive.urdf" --video --out "$W/results/V1_mj_$T" 2>&1 | grep -v "duplicate name"
    judge "$W/results/V1_mj_$T"
    echo "=== $T Isaac" | tee -a "$SUM"
    gpu bash -c "~/isaacsim-venv/bin/python sim/isaac/import_waffle_4wd.py --urdf $V/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf \
        --package-dir $V/waffle_4wd_description_261027_1711 --out $V/usd --/crashreporter/url= > $V/import.log 2>&1 && \
        ~/isaacsim-venv/bin/python 261027_1744/phys_tests/run_phys_test.py --test V1 --wheel-armature $WA --arm-armature $AA --arm-kd 36 \
        --tag v12_$T --usd $V/usd/waffle_4wd/waffle_4wd.usda --urdf $V/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf \
        --out $W/results --/crashreporter/url= > $V/V1_isaac.log 2>&1"
    R=$(grep -oE "RESULT_DIR: .+" "$V/V1_isaac.log" | tail -1 | cut -d' ' -f2)
    if [ -n "$R" ]; then judge "$R"; else echo "Isaac $T: 결과 없음($V/import.log, V1_isaac.log)" | tee -a "$SUM"; fi
done
echo "끝. 요약 $SUM" | tee -a "$SUM"
