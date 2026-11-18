#!/usr/bin/env bash
# 판정 2차에서 다시 찍은 폴더 하나만 읽는다 (Jetson). read_all.sh 로 이미 빌드한 판을 그대로 쓴다.
#   실행: bash ~/real_judge2/read_one.sh tag03   → 그 이름의 가장 새 폴더를 읽어 ~/real_judge2/results_redo/<폴더>/ 에 저장 · 표 출력
set -u
H=$HOME/real_judge2; ROS=$(ls -d /opt/ros/*/ | head -1); set +u; source ${ROS}setup.bash; source $H/install/setup.bash; set -u
D=$(ls -dt $H/clips/$1_10cm_* | head -1); N=$(basename $D); C=$H/redo_clips/$N; R=$H/results_redo/$N; mkdir -p $R $(dirname $C); ln -sfn $D $C
for f in $(ls $D/*.png | sort); do echo "$N/$(basename $f .png) $f"; done > $R/images.txt
$H/install/perception/lib/perception/barcode_compare --read $R/images.txt 1 2 4 label.enabled=1 > $R/barcode.jsonl 2> $R/barcode.err; echo "barcode_rc=$?"
$H/tag_read $R/images.txt 640 > $R/tag.jsonl 2> $R/tag.err; echo "tag_rc=$?"
python3 $H/summarize.py $H/redo_clips $R | tee $R/summary.txt
