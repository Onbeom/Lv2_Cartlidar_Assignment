#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
for posture in stow zero; do
  SELF_FILTER_POSE="$posture" flock /tmp/lv2_gpu.lock timeout --kill-after=10s 360s /home/pa4/isaacsim-venv/bin/python /home/pa4/git/lv2_next_module/261029_2329/render_holdout.py --/crashreporter/url= > "/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/camera_new_model_${posture}_holdout.log" 2>&1
done
