#!/usr/bin/env bash
D=/home/pa4/git/lv2_next_module/261104_0112_arm_lock_yolo; V=/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/a1_new_model_261029/validation_v12
for ARM in main color; do
  R=$V bash $D/run_set.sh f3_v12_$ARM $ARM
  R=$D/judge_set bash $D/run_set.sh f3_s1_$ARM $ARM
  R=$D/judge_set2 bash $D/run_set.sh f3_s2_$ARM $ARM
done
for n in f3_v12_main f3_v12_color f3_s1_main f3_s1_color f3_s2_main f3_s2_color; do
  python3 -c "
import json;s=json.load(open('$D/$n/perception_summary.json'));print('$n',{k:s[k] for k in ('P1','P2','P3_other_frames','P3_hidden','P4')})"
done
