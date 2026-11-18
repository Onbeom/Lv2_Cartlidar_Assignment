#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
flock /tmp/lv2_gpu.lock timeout --kill-after=10s 360s /home/pa4/isaacsim-venv/bin/python /home/pa4/git/lv2_next_module/261029_1408/camera_audit/render_near_controls.py --/crashreporter/url= > /home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/followup_261028_1457/camera_near_controls.log 2>&1
