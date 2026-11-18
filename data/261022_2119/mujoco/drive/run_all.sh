#!/usr/bin/env bash
# MuJoCo 주행 시험 전체 실행(PC). 결과: drive/results/<YYMMDD_HHMM>/
#   1) 기준 조건(바퀴 armature 0.0163, 팔 0.045)으로 M1·M5·M6·M7을 dt A(1/120)·B(0.002)·C(0.001)에서
#   2) 민감도: dt B에서 바퀴 armature만 0(대조)·0.00163(×0.1)·0.163(×10)으로 M5·M6·M7
#   3) 시험마다 analyze_drive.py → verdict.json, 마지막에 compare_drive.py → summary.md·summary.json
# 모든 실행에 video.mp4(640×480, 30 fps)를 남긴다(렌더링은 물리에 영향 없음).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$HERE/../.venv/bin/python"
OUT="$HERE/results/$(date +%y%m%d_%H%M)"
mkdir -p "$OUT"
WA=0.0163   # 바퀴: OP3 시스템 식별값 0.045(XM430-W350) × (212.6/353.5)²
AA=0.045    # 팔: XM430-W350 시스템 식별값 그대로

run() {  # run <시험> <dt 이름> <dt> <바퀴 armature>
    local name="$1_dt$2_wa$4"
    "$PY" "$HERE/mujoco_drive.py" "$1" --dt "$3" --wheel-armature "$4" --arm-armature "$AA" --video --out "$OUT/$name" \
        2>&1 | grep -v "WARNING: Geom with duplicate name" || true
    python3 "$HERE/analyze_drive.py" "$OUT/$name"
}

for dt in "A 0.008333333333333333" "B 0.002" "C 0.001"; do
    set -- $dt
    for t in M1 M5 M6 M7; do run "$t" "$1" "$2" "$WA"; done
done
for wa in 0 0.00163 0.163; do
    for t in M5 M6 M7; do run "$t" B 0.002 "$wa"; done
done
python3 "$HERE/compare_drive.py" "$OUT"
echo "결과 폴더: $OUT"
