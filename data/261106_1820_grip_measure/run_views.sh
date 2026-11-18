#!/bin/bash
cd /home/pa4/git/lv2_next_module/261106_1820_grip_measure
PY="flock /tmp/lv2_gpu.lock timeout 500 /home/pa4/pa-course/IsaacLab-6.1/.venv/bin/python chain_view_eval.py --num_envs 192 --seed 2"
$PY --q_view 0,-1.7,-0.1,1.28,0 --view_speed 0.25 --out /home/pa4/git/lv2_next_module/261106_1820_grip_measure/chain_D_slow.json > chain_D_slow.log 2>&1
$PY --q_view 0,-1.7,-0.2,1.64,0 --view_speed 0.25 --out /home/pa4/git/lv2_next_module/261106_1820_grip_measure/chain_A_slow.json > chain_A_slow.log 2>&1
$PY --grip_close -0.011 --out /home/pa4/git/lv2_next_module/261106_1820_grip_measure/chain_base_g.json > chain_base_g.log 2>&1
$PY --q_view 0,-1.7,-0.1,1.28,0 --view_speed 0.25 --grip_close -0.011 --out /home/pa4/git/lv2_next_module/261106_1820_grip_measure/chain_D_slow_g.json > chain_D_slow_g.log 2>&1
