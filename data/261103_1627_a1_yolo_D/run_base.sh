#!/usr/bin/env bash
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261103_1627_a1_yolo_D
R=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/validation_v12
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$ROOT/261103_1546_yolo_D_f1/branch/install/setup.bash
export A1_SET='detector.engine_path=""' A1_DOMAIN=94 A1_ARM=color
OUT=$D/R_base
REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s bash $D/run_replay_a1.sh $URDF $OUT $SETUP --render $R > $OUT.log 2>&1; echo "base replay_rc=$?" >> $D/result.log
test -s $OUT/summary.json && python3 $ROOT/261029_a1_new_model/score_perception.py $R $OUT > ${OUT}_score.log 2>&1; echo "base score_rc=$?" >> $D/result.log
