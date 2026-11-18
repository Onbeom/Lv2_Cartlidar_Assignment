#!/usr/bin/env bash
# 판정 J1~J4: 위상 6 개 × 켬(0.05) · 끔(0), TF 멈춤 15 s × 켬 · 끔, 각 35 s (PC)
set -uo pipefail
D=/home/pa4/git/lv2_next_module/261104_0056_tf_wait
{ sha256sum $0 $D/run_harness.sh $D/tf_lag_harness.py $D/analyze.py $D/judge.py; git -C $HOME/git/Lv2_Cartlidar_Assignment_tfwait rev-parse HEAD; git -C $HOME/git/Lv2_Cartlidar_Assignment_tfwait diff | sha256sum; } > $D/source_sha256_judge.txt
for p in 0.003 0.0086 0.0142 0.0198 0.0254 0.031; do
  for w in 0.05 0.0; do bash $D/run_harness.sh j_p${p}_w$w $w --duration 35 --img-phase $p; done
done
for w in 0.05 0.0; do bash $D/run_harness.sh j_stop_w$w $w --duration 35 --img-phase 0.0142 --tf-stop 15; done
python3 $D/judge.py $D/h > $D/judge_result.txt 2>&1
echo done > $D/judge_done.txt
