# P5d 다시(깨끗하게): 떠 있는 p5 스택(진단 오버레이 SIGDBG, 창 없음)에서 구간마다 [P5c2] 줄 번호를 찍는다.
#   A 없음 40 s → B raw bag 40 s → 끔 확인 → R1 없음 30 s → C compressed bag 40 s → 끔 확인 → R2 없음 30 s
#   결과 ~/real_drive/p5_out/p5d2.txt. bag 은 ~/real_drive/p5d_bag 에 쓰고 info 를 남긴 뒤 지움(시험용)
source ~/real_drive/env.sh
O=~/real_drive/p5_out; L=$O/perception.log; B=~/real_drive/p5d_bag; R=$O/p5d2.txt
n() { grep -c '\[P5c2\]' $L; }
wl() { local n0=$(n); until [ $(n) -ge $((n0+$1)) ]; do sleep 2; done; }
mark() { echo "=== $1 $(date +%T) 줄 $(n)" >> $R; }
: > $R; bash ~/real_drive/bag_kill.sh >> $R; rm -rf $B; mkdir -p $B
wl 1; mark A_start; wl 8; mark A_end
ros2 bag record -o $B/raw --topics /top_camera_front/color/image_raw > $O/p5d2_raw.log 2>&1 &
sleep 4; mark B_start; wl 8; mark B_end
bash ~/real_drive/bag_kill.sh >> $R; mark R1_start; wl 6; mark R1_end
ros2 bag record -o $B/comp --topics /top_camera_front/color/image_raw/compressed > $O/p5d2_comp.log 2>&1 &
sleep 4; mark C_start; wl 8; mark C_end
bash ~/real_drive/bag_kill.sh >> $R; mark R2_start; wl 6; mark R2_end
for b in raw comp; do echo "--- bag $b" >> $R; ros2 bag info $B/$b >> $R 2>&1; done
grep '\[P5c2\]' $L | sed 's/.*\[P5c2\]//' | nl | awk '{print "줄", $1, "앞도착", $4, "앞깊이없음", $14, "뒤도착", $18}' >> $R
rm -rf $B
echo P5D2_DONE >> $R
