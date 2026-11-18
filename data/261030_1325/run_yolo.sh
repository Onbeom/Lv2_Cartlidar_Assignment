#!/bin/bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT="$ROOT/Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325"
export YOLO_OFFLINE=true
export MPLCONFIGDIR=/tmp/lv2_plot_config
export PYTHONUNBUFFERED=1
exec 9>/tmp/lv2_gpu.lock
flock -w 1800 9
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv > "$OUT/gpu_before.csv"
/home/pa4/yolo-venv/bin/python -c 'import torch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name(0))' > "$OUT/gpu_probe.log"
for variant in A_positive B_negative; do
  timeout --signal=INT --kill-after=30s 1800 /home/pa4/yolo-venv/bin/python "$ROOT/261030_1325/train_candidate.py" "$variant" > "$OUT/train_${variant}.log" 2>&1
done
echo "두 후보 학습 종료"
