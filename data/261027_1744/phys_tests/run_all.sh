#!/usr/bin/env bash
# 물리 시험·진단 실험을 차례로 실행한다(PC). 기본은 창을 띄워 실제 시간 속도로 보여 준다.
#   창에서 시험이 끝나면 마지막 자세가 남는다. 창을 닫으면 판정 한 줄을 출력하고 다음 시험으로 넘어간다.
# 사용: bash 261027_1744/phys_tests/run_all.sh [--headless] [시험 이름 ...]
#   시험 이름: P1 P2 P3 P4on P4off P5 P6 P7 D1A D1B D2A D2B D2C (생략하면 P1~P7 전부, P6에 P8 포함)
#   EXTRA_ARGS 환경 변수의 인자를 run_phys_test.py에 그대로 넘긴다(v11 예: EXTRA_ARGS="--wheel-armature $WA --arm-armature 0.045 --tag v11")
set -u
cd "$(dirname "$0")/../.."
GUI=--gui
if [ "${1:-}" = "--headless" ]; then GUI=""; shift; fi
TESTS=("$@"); [ ${#TESTS[@]} -eq 0 ] && TESTS=(P1 P2 P3 P4on P4off P5 P6 P7)
D=261027_1744/phys_tests
source 261027_1744/conditions.sh
mkdir -p $D/logs $D/results
for T in "${TESTS[@]}"; do
  LOG=$D/logs/${T}_$(date +%y%m%d_%H%M%S).log
  echo "=== $T 시작 (로그 $LOG)"
  ~/isaacsim-venv/bin/python $D/run_phys_test.py --test "$T" $GUI ${EXTRA_ARGS:-} \
    --usd "$USD" \
    --urdf "$URDF" \
    --out $D/results --/crashreporter/url= 2>&1 | tee "$LOG" | grep --line-buffered -E "구간 시작|t= |시험 끝|RESULT_DIR|STEPS|Traceback|Error:"
  R=$(grep -oE "$D/results/[^ ]+" "$LOG" | tail -1)
  if [ -n "$R" ]; then
    python3 $D/analyze_phys_test.py "$R" > /dev/null 2>&1
    python3 -c "import json,sys; d=json.load(open(sys.argv[1]+'/verdict.json')); v=d.get('verdict') or ' / '.join(k+' '+d[k]['verdict'] for k in ('P6','P8')); print('판정:', v)" "$R"
  else
    echo "판정: 결과 폴더 없음(로그 확인)"
  fi
done
