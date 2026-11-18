#!/bin/bash
M=$HOME/git/lv2_next_module/261107_1742_manip_rl_src/lv2_urdfusd/policy/manipulator
OUT=$HOME/git/lv2_next_module/261106_1820_grip_measure/fit1_runs
PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
export HEADLESS=1
cd $M
run() { n=$1; sh=$2; shift 2
  if [ $sh = cyl ]; then P="--sidepick $M/policies/sidepick45_fit1.pt"; else P="--box_prob 1.0 --sidepick $M/policies/sidepick45box_fit1.pt"; fi
  flock /tmp/lv2_gpu.lock timeout 1200 $PY chain45_view.py --num_envs 96 --per_box 2 --fill_order lr --occ 1.0 --seconds 70 $P "$@" > $OUT/${n}_$sh.log 2>&1
  echo "$n $sh 끝 $(date +%H:%M)"
}
for sh in cyl box; do
  run gate $sh --est --close_gate 0.006 --freeze_close 0.4
  run far_meas1 $sh --est --q_view 0,-0.2,1.55,-1.87,0 --view_speed 0.5 --view_meas 1
  run far_meas2 $sh --est --q_view 0,-0.2,1.55,-1.87,0 --view_speed 0.5 --view_meas 2
done
