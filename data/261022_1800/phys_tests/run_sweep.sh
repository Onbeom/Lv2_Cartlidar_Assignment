#!/usr/bin/env bash
# 이득 조정 G(팔 감쇠)와 마찰 진단 D4(P7 마찰)를 창 없이 차례로 실행한다(PC).
# 사용: bash 261022_1800/phys_tests/run_sweep.sh G | D4
set -u
cd "$(dirname "$0")/../.."
D=261022_1800/phys_tests
run() {  # run <시험> <태그> [추가 인자...]
  local T=$1 TAG=$2; shift 2
  local LOG=$D/logs/${T}_${TAG}_$(date +%y%m%d_%H%M%S).log
  ~/isaacsim-venv/bin/python $D/run_phys_test.py --test "$T" --tag "$TAG" "$@" \
    --usd $D/usd_copy/waffle_4wd/waffle_4wd.usda \
    --urdf 261022_1800/urdf/waffle_4wd_description_261022_1800/urdf/waffle_4wd.urdf \
    --out $D/results --/crashreporter/url= > "$LOG" 2>&1
  local R=$(grep -oE "$D/results/[^ ]+" "$LOG" | tail -1)
  if [ -n "$R" ]; then python3 $D/analyze_phys_test.py "$R" > /dev/null 2>&1
    echo "$T $TAG $(python3 -c "import json,sys;d=json.load(open(sys.argv[1]+'/verdict.json'));print(d.get('verdict') or ' / '.join(k+' '+d[k]['verdict'] for k in ('P6','P8')))" "$R") $R"
  else echo "$T $TAG 결과없음 $LOG"; fi
}
case "${1:-}" in
  G)  for KD in 2 4 8 16; do run P2 kd$KD --arm-kd $KD; run P4off kd$KD --arm-kd $KD; done ;;
  G2) for KPD in "200 12" "400 16" "400 24"; do set -- $KPD; run P2 kp$1kd$2 --arm-kp $1 --arm-kd $2; run P4off kp$1kd$2 --arm-kp $1 --arm-kd $2; done ;;
  D4) for MU in "0.8 0.6" "0.6 0.45" "0.4 0.3" "0.2 0.15"; do run P7 mu${MU%% *} --mu $MU; done ;;
  *) echo "사용: $0 G|G2|D4"; exit 1 ;;
esac
