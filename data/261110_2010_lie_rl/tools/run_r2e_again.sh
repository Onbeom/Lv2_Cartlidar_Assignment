#!/usr/bin/env bash
# r2e 다시 (모의, 11-11 04:4x): 첫 r2e 는 NaN 으로 1721 에서 멈추고 저장이 1500 (= r2d + 2 반복) 뿐이라 B4 시험이 안 됨 -> NaN 보호 넣고 r2d 1498 에서 500 반복 -> 3 seed -> 채택 판정 -> 필요하면 r2f
set -u
W="$(cd "$(dirname "$0")/.." && pwd)"
bash "$W/tools/run_r2e.sh"
bash "$W/tools/run_r2f_if.sh"
