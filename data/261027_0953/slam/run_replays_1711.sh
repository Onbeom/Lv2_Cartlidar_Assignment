#!/usr/bin/env bash
# 261027_1711 본 기록 6개(results/m1711_*) 재생 묶음 (PC, 모의, 사용자 승인 2026-10-27 20:2x):
#   cuVSLAM V0 · V1 · V2(GPU 잠금, 도메인 27). Cartographer(CPU, 도메인 28)는 따로 먼저 돌림.
#   끝나면 평가(eval_slam.py) → results/eval_m1711/
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(dirname "$HERE")"
RECS=$(ls -d "$WORK"/results/m1711_*_2610* | sort)
# Cartographer 는 GPU 를 안 써서 먼저 따로 돌림(2026-10-27 21:1x, 도메인 28 · logs/m1711_carto.log). 평가 전에 남은 Carto 가 끝나길 기다림
for v in V0 V1 V2; do
  for r in $RECS; do CUVSLAM_VARIANT=$v bash "$HERE/run_cuvslam.sh" "$r"; done
done > "$WORK/logs/m1711_cuvslam.log" 2>&1
while pgrep -f "run_carto.sh $WORK/results/m1711_" > /dev/null || pgrep -f "run_carto.sh results/m1711_" > /dev/null; do sleep 10; done
source /opt/ros/lyrical/setup.bash
~/slam-eval-venv/bin/python "$HERE/eval_slam.py" $RECS --out "$WORK/results/eval_m1711" > "$WORK/logs/m1711_eval.log" 2>&1
echo "[replays] 끝"
