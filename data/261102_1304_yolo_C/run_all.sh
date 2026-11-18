#!/bin/bash
# 후보 C: ONNX 내보내기 · PT↔ONNX 동등성(CPU) → GPU 잠금 안에서 TensorRT 동등성. PC.
set -o pipefail
HERE=/home/pa4/git/lv2_next_module/261102_1304_yolo_C
PY=/home/pa4/yolo-venv/bin/python
sha256sum $HERE/export_c.py $HERE/check_trt.py $HERE/run_all.sh > $HERE/source_sha256.txt
$PY $HERE/export_c.py > $HERE/export_c.log 2>&1; echo "export_exit $?" > $HERE/exit_codes.txt
exec 9>/tmp/lv2_gpu.lock; flock 9
$PY $HERE/check_trt.py > $HERE/check_trt.log 2>&1; echo "trt_exit $?" >> $HERE/exit_codes.txt
flock -u 9
echo done > $HERE/done.txt
