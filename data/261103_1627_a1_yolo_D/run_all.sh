#!/usr/bin/env bash
# A1 재판정(모의, PC): fix/perception-yolo-d 빌드 + YOLO D PC 엔진. R0 = arm.detector color(PR 그대로), R1 = main(팔도 YOLO).
# 자료 = v12 판정용 별도 시야(rng20261229, 6월드 × 조명 2). 재생 · 채점 도구는 v12 판정과 같음.
set -uo pipefail
ROOT=/home/pa4/git/lv2_next_module; D=$ROOT/261103_1627_a1_yolo_D
R=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/validation_v12
URDF=$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf
SETUP=$ROOT/261103_1546_yolo_D_f1/branch/install/setup.bash
ENGINE=$ROOT/261103_1546_yolo_D_f1/trt/yolo11n_task4_D_fp32_notf32.engine
export A1_SET="detector.engine_path=$ENGINE" A1_DOMAIN=94
sha256sum $0 $D/run_replay_a1.sh $ROOT/261029_a1_new_model/replay.py $ROOT/261029_a1_new_model/score_perception.py $ENGINE $R/render_a1.json > $D/source_sha256.txt
git -C $HOME/git/Lv2_Cartlidar_Assignment_yoloD rev-parse HEAD >> $D/source_sha256.txt
git -C $HOME/git/Lv2_Cartlidar_Assignment_yoloD diff | sha256sum >> $D/source_sha256.txt
exec 8>/tmp/lv2_gpu.lock; flock 8
for arm in color main; do
  OUT=$D/R_$arm
  A1_ARM=$arm REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s \
    bash $D/run_replay_a1.sh $URDF $OUT $SETUP --render $R > $OUT.log 2>&1; echo "$arm replay_rc=$?" >> $D/result.log
  test -s $OUT/summary.json && python3 $ROOT/261029_a1_new_model/score_perception.py $R $OUT > ${OUT}_score.log 2>&1; echo "$arm score_rc=$?" >> $D/result.log
done
flock -u 8
echo done > $D/done.txt
