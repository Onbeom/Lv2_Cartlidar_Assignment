#!/bin/bash
# GPU 잠금 안에서 TensorRT 동등성 확인. PC.
set -o pipefail
HERE=/home/pa4/git/lv2_next_module/261031_0100_trt
exec 9>/tmp/lv2_gpu.lock; flock 9
sha256sum $HERE/check_trt.py $HERE/run_trt.sh > $HERE/source_sha256.txt
/home/pa4/yolo-venv/bin/python $HERE/check_trt.py > $HERE/check_trt.log 2>&1
echo "exit $?" >> $HERE/check_trt.log
flock -u 9
