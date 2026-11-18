# P5 실습실 판정 (PC 에서 실행): 신호등 프레임 판정(P5a) + 12 회 순서 판정(P5b) 을 한 번에.
#   bash p5_lab.sh <결과 폴더> [판 수=3] [판마다 초=110]
#   환경 값: OVERLAY=~/real_drive/fix_ws/<이름> (Jetson 시험용 오버레이, 비우면 Jetson 설치본), TILT_MAX=2 [°],
#            BAG=0 이면 영상 bag 을 안 씀 (bag 을 쓰면 모름 메시지가 늘었음: 12.9 · 20.0 % 대 bag 없음 5~7 %, 11-10)
# 순서: Jetson 설치본 커밋 기록 → p5 스택(카메라 · 인지, 창 없음) → 기울기 확인(넘으면 멈춤) → 앞 컬러 compressed bag 시작
#       → 시계 차이 → 판마다 [인지 신호등 초기화 → 연습 신호등 새 순서 + 정답 기록] → 시계 차이 → bag 끔 → 기록 복사 → 분석
# 전제: 연습 신호등 OpenCR 은 PC /dev/ttyACM0, Jetson ssh 공유 소켓 ~/.ssh/lv2_jetson.sock, 신호등은 앞 D435 앞 약 0.66 m.
#       다른 bringup(로봇 전체)이 떠 있으면 먼저 끌 것. 끝나도 스택은 떠 있음: ssh cart11@cart11.local 'bash ~/real_drive/p5.sh stop'
set -u
D=$1; N=${2:-3}; S=${3:-110}
H=/home/pa4/git/lv2_next_module/261106_1655_real_drive
J="ssh -o ControlPath=$HOME/.ssh/lv2_jetson.sock cart11@cart11.local"
ssh -O check -o ControlPath=$HOME/.ssh/lv2_jetson.sock cart11@cart11.local 2>/dev/null || {
  echo "Jetson ssh 공유 연결이 없음 — PC 에서 먼저 한 번 (비밀번호 한 번 입력):"
  echo "  ssh -M -S ~/.ssh/lv2_jetson.sock -o ControlPersist=yes -fN cart11@cart11.local"
  exit 1; }
[ -e /dev/ttyACM0 ] || { echo "연습 신호등 OpenCR 이 PC /dev/ttyACM0 에 없음 — PC USB 에 꽂을 것"; exit 1; }
mkdir -p $D
echo "== 설치본 · 오버레이" | tee $D/run.log
$J 'git -C ~/Lv2_Cartlidar_Assignment log --oneline -1; git -C ~/Lv2_Cartlidar_Assignment status --short | head -5' | tee -a $D/run.log
echo "OVERLAY=${OVERLAY:-없음}" | tee -a $D/run.log
for f in p5.sh p5_probe.py floor_tilt.py bag_kill.sh; do scp -q -o ControlPath=$HOME/.ssh/lv2_jetson.sock $H/jetson/$f cart11@cart11.local:real_drive/; done
echo "== 스택" | tee -a $D/run.log
$J 'bash ~/real_drive/bag_kill.sh >/dev/null; bash ~/real_drive/p5.sh stop >/dev/null 2>&1; pkill -INT -f "[v]iew_front.py"; true'   # 이전 스택 · 기록기 · 미리보기 창 정리
$J "OVERLAY=${OVERLAY:-} bash ~/real_drive/p5.sh 2>&1 | tail -4" | tee -a $D/run.log
echo "== 기울기 (기준 |앞뒤| · |좌우| ≤ ${TILT_MAX:-2}°)" | tee -a $D/run.log
$J 'source ~/real_drive/env.sh >/dev/null 2>&1; python3 ~/real_drive/floor_tilt.py 6' | tee $D/tilt.txt | tee -a $D/run.log
python3 - "$D/tilt.txt" "${TILT_MAX:-2}" <<'EOF' || { echo "기울기 기준 넘음 — 카메라 · 차체를 바로잡고 다시 (스택은 떠 있음)" | tee -a $D/run.log; exit 1; }
import re, sys
s = open(sys.argv[1]).read(); m = float(sys.argv[2])
a = [float(x) for x in re.findall(r'앞뒤 ([+-][\d.]+)°.*좌우 ([+-][\d.]+)°', s)[0]]
sys.exit(0 if abs(a[0]) <= m and abs(a[1]) <= m else 1)
EOF
B=$(basename $D)
if [ "${BAG:-1}" != 0 ]; then
echo "== 영상 bag (앞 컬러 compressed)" | tee -a $D/run.log
$J "source ~/real_drive/env.sh >/dev/null 2>&1; mkdir -p ~/real_drive/p5_lab_bag; rm -rf ~/real_drive/p5_lab_bag/$B; nohup ros2 bag record -o ~/real_drive/p5_lab_bag/$B --topics /top_camera_front/color/image_raw/compressed /perception/signal > ~/real_drive/p5_out/p5_lab_bag.log 2>&1 < /dev/null & sleep 4; tail -1 ~/real_drive/p5_out/p5_lab_bag.log"
fi
python3 $H/jetson/clock_offset.py 10 > $D/clock_before.json; cat $D/clock_before.json | tee -a $D/run.log
echo "== 판 $N 개 × $S s" | tee -a $D/run.log
bash $H/jetson/p5e_pc.sh $D $N $S | tail -1
python3 $H/jetson/clock_offset.py 10 > $D/clock_after.json; cat $D/clock_after.json | tee -a $D/run.log
$J 'bash ~/real_drive/bag_kill.sh' | tee -a $D/run.log
[ "${BAG:-1}" != 0 ] && $J "source ~/real_drive/env.sh >/dev/null 2>&1; ros2 bag info ~/real_drive/p5_lab_bag/$B 2>&1 | grep -E 'Duration|Count'" | tee -a $D/run.log
for f in p5.jsonl perception.log sensors.log; do scp -q -o ControlPath=$HOME/.ssh/lv2_jetson.sock cart11@cart11.local:real_drive/p5_out/$f $D/; done
echo "== P5b 순서" | tee -a $D/run.log
python3 $H/jetson/p5e_analyze.py $D $D/p5.jsonl | tee $D/p5b.txt | tee -a $D/run.log
echo "== P5a 프레임" | tee -a $D/run.log
python3 $H/jetson/p5a_frames.py $D $D/p5.jsonl | tee $D/p5a.txt | tee -a $D/run.log
[ "${BAG:-1}" != 0 ] && echo "bag(영상)은 Jetson ~/real_drive/p5_lab_bag/$B 에 있음 (사람 얼굴이 찍혔을 수 있어 팀 저장소에 넣지 않음)" | tee -a $D/run.log
echo P5_LAB_DONE
