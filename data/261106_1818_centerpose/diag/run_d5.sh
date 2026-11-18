#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1818_centerpose
export PYTHONDONTWRITEBYTECODE=1
exec 9>/tmp/lv2_gpu.lock; flock -w 3600 9
timeout 1500 /home/pa4/isaacsim-venv/bin/python diag/d5_retrain_settings.py cuboid 4 5e-4 8 30 > diag/d5_cuboid.log 2>&1
timeout 1500 /home/pa4/isaacsim-venv/bin/python diag/d5_retrain_settings.py cylinder 12 5e-4 8 30 > diag/d5_cylinder.log 2>&1
