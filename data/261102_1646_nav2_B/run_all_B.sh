#!/bin/bash
# 차례로: 설계 45 B안 판정 3 회(GPU 잠금 기다림) → K2e(통과 때만) → K0 · K1 감지 개선(통과 때만). 끝나면 results/ALL_done.txt. PC.
H=/home/pa4/git/lv2_next_module/261102_1646_nav2_B
bash $H/run_reps_B.sh live_nav_v45B3 $H/config/nav2_params_v45_B.yaml 20261101 1 2
bash $H/run_k2e_B.sh
bash $H/run_k01_loc2_B.sh
date > $H/results/ALL_done.txt
