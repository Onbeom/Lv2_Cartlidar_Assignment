#!/usr/bin/env bash
# 판정 2차 (Jetson, 실물): 새 판(dev 30c2d1f + 바코드 라벨 찾기 + 태그 줄여 찾기) 인지를 빌드해, 새로 찍은 장을 모두 읽고 B1~B3 · T1~T3 표를 낸다.
#   실행 (Jetson): bash ~/real_judge2/read_all.sh     → ~/real_judge2/results/ · summary.txt
set -u
H=$HOME/real_judge2; C=$H/clips; R=$H/results; mkdir -p $R; cd $H || exit 1
ROS=$(ls -d /opt/ros/*/ 2>/dev/null | head -1); set +u; source ${ROS}setup.bash; set -u
rm -rf workspace build install log; tar xzf src_judge2.tar.gz   # 묶음을 새로 받아도 옛 빌드를 쓰지 않게 매번 새로 빌드 (촬영 영상 clips 는 그대로)
colcon build --base-paths workspace/ros2_ws/src --packages-up-to perception --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF > build.log 2>&1
echo "build_rc=$?"
set +u; source install/setup.bash; set -u
BC=$H/install/perception/lib/perception/barcode_compare
[ -x $BC ] || { echo "barcode_compare 없음: build.log 확인"; exit 1; }
g++ -std=c++17 -O2 -DTAG_READ_MAX_WIDTH -o $H/tag_read $H/tag_read.cpp -I$H/workspace/ros2_ws/src/perception/include \
  $H/build/perception/libperception_detector.a $(pkg-config --cflags --libs opencv4) || { echo "tag_read 빌드 실패"; exit 1; }
: > $R/images.txt
for d in $(ls -d $C/*/ 2>/dev/null | sort); do
  n=$(basename $d)
  for f in $(ls $d/*.png 2>/dev/null | sort); do echo "$n/$(basename $f .png) $f" >> $R/images.txt; done
done
echo "장 수: $(wc -l < $R/images.txt) (폴더 $(ls -d $C/*/ 2>/dev/null | wc -l)개)"
$BC --read $R/images.txt 1 2 4 label.enabled=1 > $R/barcode.jsonl 2> $R/barcode.err; echo "barcode_rc=$?"
$H/tag_read $R/images.txt 640 > $R/tag.jsonl 2> $R/tag.err; echo "tag_rc=$?"
python3 $H/summarize.py $C $R | tee $R/summary.txt
