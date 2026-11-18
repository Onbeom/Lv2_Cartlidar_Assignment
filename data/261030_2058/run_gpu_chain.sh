#!/bin/bash
# 녹화 3회(261030_1943/run_record_reps.sh)가 끝나면 GPU 작업을 차례로: TensorRT 동등성 → cuVSLAM(밝음 · 어두움 × V0 · V1 260행 가림) → 채점.
# 각 작업이 /tmp/lv2_gpu.lock 을 따로 잡음. PC.
set -u
TOP=/home/pa4/git/lv2_next_module
HERE=$TOP/261030_2058
R=$TOP/261030_1943/results
until grep -qx done $TOP/261030_1943/results_reps.log 2>/dev/null; do sleep 30; done
$TOP/261031_0100_trt/run_trt.sh; echo "[gpu] trt 끝 $(tail -1 $TOP/261031_0100_trt/check_trt.log)"
sha256sum $HERE/cuvslam/* $HERE/evaluate_cuvslam.py $HERE/run_gpu_chain.sh > $HERE/results/source_sha256_cuvslam.txt
for spec in bright:n261028_bright_261030_211830 dark:n261028_dark_261030_213001; do
  label=${spec%%:*}; rec=$R/${spec#*:}
  CUVSLAM_VARIANT=V0 $HERE/cuvslam/run_cuvslam.sh $rec $label 1.0; echo "[gpu] cuvslam $label V0 끝 $?"
  CUVSLAM_VARIANT=V1 MASK_BOTTOM_PX=260 $HERE/cuvslam/run_cuvslam.sh $rec $label 1.0; echo "[gpu] cuvslam $label V1 끝 $?"
done
( set +u; source /opt/ros/lyrical/setup.bash; export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-}
  /home/pa4/isaacsim-venv/bin/python $HERE/evaluate_cuvslam.py bright=$R/n261028_bright_261030_211830 dark=$R/n261028_dark_261030_213001 ) > $HERE/results/evaluate_cuvslam.log 2>&1
echo done
