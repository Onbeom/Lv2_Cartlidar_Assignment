#!/usr/bin/env bash
# tf_lag_harness 한 판: run_harness.sh <이름> <tf_wait_s> [harness 인자...]  (PC, F1 과 겹치지 않게 GPU 잠금 뒤)
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261104_0056_tf_wait
NAME=$1; WAIT=$2; shift 2
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
RENDER=$ROOT/261031_1718_r3port/b0/judge/render_101_T3000_awb
set +u; source /opt/ros/lyrical/setup.bash; set -u
export REPLAY_DOMAIN=139 REPLAY="../../261104_0056_tf_wait/tf_lag_harness.py"
export REPLAY_SET="ros__parameters.tf_wait_s=$WAIT"
exec 9>/tmp/lv2_gpu.lock; flock 9
bash $ROOT/Cartlidar/sim_render/run_replay.sh $URDF $D/h/$NAME $D/build/install/setup.bash -- --render $RENDER "$@" > $D/h/$NAME.log 2>&1
python3 $D/analyze.py $D/h/$NAME $(printf '%s\n' "$@" | grep -A1 -- --tf-stop | tr '\n' ' ') | tee -a $D/h/results.jsonl
