#!/usr/bin/env bash
# 설계 자료(이미 쓴 v12 A1 묶음): 이 브랜치 빌드로 R_main_settle 과 같은 재생 + ARM_LOCK 진단 줄. run_diag.sh <이름> [color|main]
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261104_0112_arm_lock_yolo
NAME=$1; ARM=${2:-main}
R=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/validation_v12
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$D/build/install/setup.bash
ENGINE=$ROOT/261103_1546_yolo_D_f1/trt/yolo11n_task4_D_fp32_notf32.engine
export A1_SET="detector.engine_path=$ENGINE" A1_DOMAIN=94 A1_ARM=$ARM LOCK_LOG=1
OUT=$D/$NAME
exec 8>/tmp/lv2_gpu.lock; flock 8
REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s bash $D/run_replay_a1.sh $URDF $OUT $SETUP --render $R --settle 0.2 > $OUT.log 2>&1; echo "$NAME replay_rc=$?" >> $D/result.log
test -s $OUT/summary.json && python3 $ROOT/261029_a1_new_model/score_perception.py $R $OUT > ${OUT}_score.log 2>&1; echo "$NAME score_rc=$?" >> $D/result.log
