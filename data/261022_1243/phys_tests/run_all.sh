#!/usr/bin/env bash
# 물리 시험 P1~P7(P6에 P8 포함)을 차례로 실행한다(PC). 각 시험은 새 프로세스.
# 사용: bash 261022_1243/phys_tests/run_all.sh [시험 이름 ...]   (생략하면 전부)
set -u
cd "$(dirname "$0")/../.."
TESTS=("$@"); [ ${#TESTS[@]} -eq 0 ] && TESTS=(P1 P2 P3 P4on P4off P5 P6 P7)
mkdir -p 261022_1243/phys_tests/logs 261022_1243/phys_tests/results
for T in "${TESTS[@]}"; do
  LOG=261022_1243/phys_tests/logs/${T}_$(date +%y%m%d_%H%M%S).log
  ~/isaacsim-venv/bin/python 261022_1243/phys_tests/run_phys_test.py --test "$T" \
    --usd 261022_1243/phys_tests/usd_copy/waffle_4wd/waffle_4wd.usda \
    --urdf 261022_1243/urdf/waffle_4wd_description_261022/urdf/waffle_4wd.urdf \
    --out 261022_1243/phys_tests/results --/crashreporter/url= > "$LOG" 2>&1
  echo "$T exit=$? $(grep -E 'RESULT_DIR|STEPS' "$LOG" | tr '\n' ' ')"
done
