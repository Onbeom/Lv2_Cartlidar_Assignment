#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
export OPENBLAS_NUM_THREADS=1
for posture in stow zero; do
 export SELF_FILTER_SOURCE="camera_new_model_${posture}_holdout"
 export SELF_FILTER_REFERENCE="nvblox_self_filter_${posture}_holdout"
 export SELF_FILTER_CACHED="nvblox_self_filter_cached_${posture}_holdout"
 /home/pa4/isaacsim-venv/bin/python /home/pa4/git/lv2_next_module/261029_2329/filter_check.py
 /home/pa4/isaacsim-venv/bin/python /home/pa4/git/lv2_next_module/261029_2329/filter_cached.py
done
