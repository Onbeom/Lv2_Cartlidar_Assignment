#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
OUT=$ROOT/Cartlidar/sim_render/results/camera_calibration_261029
ssh -S /tmp/lv2_camera_ssh.sock -o BatchMode=yes cart11@cart11.local 'timeout 30s python3 -' < "$ROOT/261029_camera_calibration/preview_remote.py" > "$OUT/arducam_preview.jpg" 2> "$OUT/arducam_preview_metadata.log"
test -s "$OUT/arducam_preview.jpg"
cat "$OUT/arducam_preview_metadata.log"
