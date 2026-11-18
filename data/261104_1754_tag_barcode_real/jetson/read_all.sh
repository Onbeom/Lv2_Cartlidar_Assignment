#!/usr/bin/env bash
# 찍은 장을 모두 팀 코드로 읽는다 (Jetson): 바코드 = barcode_compare --read(인지 설정 read_scale 1 · gamma 2 · quality 4),
#   태그 = tag_read(perception::TagDetector tag36h11). 모든 장을 두 가지로 다 읽어 다른 것을 잘못 읽는지도 본다.
#   실행 (Jetson): bash ~/tag_barcode_check/read_all.sh      → ~/tag_barcode_check/results/ · summary.txt
set -u
H=$HOME/tag_barcode_check; C=$H/clips; R=$H/results
P=$HOME/e2_engine_check                     # 8fcf9f8(PR #103) 인지를 빌드해 둔 곳 (Jetson 기동 확인)
mkdir -p $R
BC=$P/install/perception/lib/perception/barcode_compare
[ -x $BC ] || { echo "barcode_compare 없음: $BC (e2_engine_check 빌드 먼저)"; exit 1; }
g++ -std=c++17 -O2 -o $H/tag_read $H/tag_read.cpp -I$P/workspace/ros2_ws/src/perception/include \
  $P/build/perception/libperception_detector.a $(pkg-config --cflags --libs opencv4) || { echo "tag_read 빌드 실패"; exit 1; }
: > $R/images.txt
for d in $(ls -d $C/*/ 2>/dev/null | sort); do
  n=$(basename $d)
  for f in $(ls $d/*.png 2>/dev/null | sort); do echo "$n/$(basename $f .png) $f" >> $R/images.txt; done
done
echo "장 수: $(wc -l < $R/images.txt) (폴더 $(ls -d $C/*/ 2>/dev/null | wc -l)개)"
$BC --read $R/images.txt 1 2 4 > $R/barcode.jsonl 2> $R/barcode.err; echo "barcode_rc=$?"
$H/tag_read $R/images.txt > $R/tag.jsonl 2> $R/tag.err; echo "tag_rc=$?"
python3 $H/summarize.py $C $R | tee $R/summary.txt
