# P5d: 헤드리스 p5 스택(진단 오버레이 SIGDBG) + bag 없음 · raw · compressed 각 약 40 s 의 [P5c2] 앞 · 뒤 도착 수.
#   bag 은 ~/real_drive/p5d_bag/ 에 쓰고 재고 나면 지운다(시험용, 용량 큼). 끝나도 스택은 떠 있음: bash ~/real_drive/p5.sh stop
O=~/real_drive/p5_out
L=$O/perception.log
B=~/real_drive/p5d_bag
[ "${1:-}" = nostart ] || OVERLAY=~/real_drive/fix_ws/SIGDBG FRONT=261822072247 REAR=261822079801 bash ~/real_drive/p5.sh | tail -4
source ~/real_drive/env.sh
mark() { echo "=== $1 $(date +%T) 줄 $(grep -c '\[P5c2\]' $L)"; }
wait_lines() { local n0=$(grep -c '\[P5c2\]' $L); until [ $(grep -c '\[P5c2\]' $L) -ge $((n0+$1)) ]; do sleep 2; done; }
until grep -q "\[P5c2\]" $L; do sleep 2; done   # 고정된 뒤부터 계수가 나옴
wait_lines 1
rm -rf $B; mkdir -p $B
mark A_start; wait_lines 8; mark A_end
ros2 bag record -o $B/raw --topics /top_camera_front/color/image_raw > $O/p5d_raw.log 2>&1 &
sleep 4; mark B_start; wait_lines 8; mark B_end
pkill -INT -f "[b]ag record -o $B/raw"; sleep 4
ros2 bag record -o $B/comp --topics /top_camera_front/color/image_raw/compressed > $O/p5d_comp.log 2>&1 &
sleep 4; mark C_start; wait_lines 8; mark C_end
pkill -INT -f "[b]ag record -o $B/comp"; sleep 4
ros2 bag info $B/raw 2>&1 | grep -E "Duration|Messages|Size"; ros2 bag info $B/comp 2>&1 | grep -E "Duration|Messages|Size"
grep '\[P5c2\]' $L | sed 's/.*\[P5c2\]//' | cut -c1-150 | nl > $O/p5d_counts_${1:-run}.txt
rm -rf $B
echo P5D_DONE
