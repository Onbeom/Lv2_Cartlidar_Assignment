#!/bin/bash
# 후방 장애물 재생 · 바코드 C++↔Python 대조 (PC, 모의 · 도구 확인)
D=/home/pa4/git/lv2_next_module/261101_0510_stale3
R=/home/pa4/git/lv2_next_module/Cartlidar/sim_render
I=$D/build_merged/install
bash $D/run_replay_rear.sh /home/pa4/git/lv2_next_module/261027_2058/results/judge_render $D/rear_replay $I/setup.bash > $D/rear_replay.log 2>&1
echo "rear rc=$?" >> $D/checks.log
set +u; source /opt/ros/lyrical/setup.bash; source $I/setup.bash
systemd-run --user --wait -p MemoryMax=12G -p MemorySwapMax=0 --collect \
  python3 -I $HOME/git/Lv2_Cartlidar_Assignment_merge16/workspace/ros2_ws/src/perception/scripts/barcode_compare.py \
  --ref $R/proto_r3/locate_v2.py --tool $I/perception/lib/perception/barcode_compare \
  --tex $HOME/git/Lv2_Cartlidar_Assignment_merge16/workspace/ros2_ws/src/perception/data/barcode_P2-01.png \
  --render $R/results/render_r3_judge7_261026 --cond clean n0.0018c3s0 n0.0018c3s1 n0.0018c3s2 --out $D/barcode_cmp --jobs 6 > $D/barcode_cmp.log 2>&1
echo "barcode rc=$?" >> $D/checks.log
echo done >> $D/checks.log
