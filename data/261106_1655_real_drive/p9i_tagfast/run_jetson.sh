# Jetson 혼자(카메라 · 인지 끔): 기존 / 새 검색을 실물 클립으로 비교. 복사본만 씀
cd ~/real_drive/tag_fast
FL="-O3 -DNDEBUG -std=c++17 $(pkg-config --cflags --libs opencv4)"
g++ -DOLD -Iold/inc replay.cpp old/tag_detector.cpp -o old/replay $FL 2>&1 | grep -E "error" ; echo "old rc ${PIPESTATUS[0]}"
g++ -Inew/inc -Wall replay.cpp new/tag_detector.cpp -o new/replay $FL 2>&1 | grep -E "error|warning" ; echo "new rc ${PIPESTATUS[0]}"
L=~/real_drive/tag_bench/clips.txt
awk 'NR%4==0' $L > s1.txt
echo "시작 $(date +%T), 표본 $(wc -l < s1.txt) 전체 $(wc -l < $L)"
old/replay find < s1.txt > r_old_s1.txt; echo "old 끝 $(date +%T)"
new/replay find < s1.txt > r_new_s1.txt; echo "new 끝 $(date +%T)"
new/replay find < $L > r_new_all.txt; echo "new 전체 끝 $(date +%T)"
new/replay track < $L > r_track_all.txt; echo "추적 끝 $(date +%T)"
echo "== 기존 대 새 전체 검색 (4 장에 1 장)"; python3 compare.py r_old_s1.txt r_new_s1.txt 새검색
echo "== 새 전체 검색 대 추적 (연속 전체 영상)"; python3 compare.py r_new_all.txt r_track_all.txt 추적
echo REPLAY_DONE
