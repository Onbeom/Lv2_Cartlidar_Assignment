#!/bin/bash
# #128 fit1 정책으로 다시: 가정 / 집기 끝에서 잼(참고) / 보는 자세 들름 / 보는 자세 + 잼
M=$HOME/git/lv2_next_module/261107_1742_manip_rl_src/lv2_urdfusd/policy/manipulator
OUT=$HOME/git/lv2_next_module/261106_1820_grip_measure/fit1_runs
PY=$HOME/pa-course/IsaacLab-6.1/.venv/bin/python
export HEADLESS=1
cd $M
run() { # 이름 모양 옵션...
  n=$1; sh=$2; shift 2
  if [ $sh = cyl ]; then P="--sidepick $M/policies/sidepick45_fit1.pt"; else P="--box_prob 1.0 --sidepick $M/policies/sidepick45box_fit1.pt"; fi
  flock /tmp/lv2_gpu.lock timeout 1200 $PY chain45_view.py --num_envs 96 --per_box 2 --fill_order lr --occ 1.0 --seconds 50 $P "$@" > $OUT/${n}_$sh.log 2>&1
  echo "$n $sh 끝 $(date +%H:%M)"
}
for sh in cyl box; do
  run assume $sh --est
  run meas1 $sh --est --est_meas 1
  run view $sh --est --q_view 0,-1.7,-0.1,1.28,0
  run view_meas1 $sh --est --q_view 0,-1.7,-0.1,1.28,0 --view_meas 1
  run view_meas2 $sh --est --q_view 0,-1.7,-0.1,1.28,0 --view_meas 2
done
for sh in cyl box; do
  run freeze $sh --est --freeze_close 0.4
  run freeze_slow $sh --est --freeze_close 0.4 --near_scale 0.3
done
