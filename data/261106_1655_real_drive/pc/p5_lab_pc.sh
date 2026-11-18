# P5 실습실 판정 (PC 혼자): D435 두 대 · 연습 신호등 OpenCR 모두 PC USB. 신호등 프레임 판정(P5a) + 12 회 순서 판정(P5b).
#   bash p5_lab_pc.sh <결과 폴더> [판 수=3] [판마다 초=110]
#   환경 값: TILT_MAX=2 [°], BAG=0 이면 영상 bag 을 안 씀 (bag 을 쓰면 Jetson 에서 모름 메시지가 늘었음 12.9 · 20.0 % 대 5~7 %)
# 순서: 장치 확인 → 팀 dev 빌드 커밋 기록 → p5 스택(창 없음) → 기울기 확인(넘으면 멈춤) → (영상 bag)
#       → 판마다 [인지 신호등 초기화 → 연습 신호등 새 순서 + 정답 기록] → bag 끔 → 분석. 시계는 PC 하나라 시계 차이 0.
# 배치: 로봇 중심(네 바퀴 가운데)에서 신호등 기둥까지 정면 약 0.65 m (미션 출발 자세와 같음), 같은 바닥, 커튼 닫음.
# 끝나도 스택은 떠 있음: bash p5_pc.sh stop
D=$1; N=${2:-3}; S=${3:-110}
H=/home/pa4/git/lv2_next_module/261106_1655_real_drive
[ -e /dev/ttyACM0 ] || { echo "연습 신호등 OpenCR 이 /dev/ttyACM0 에 없음 — PC USB 에 꽂을 것"; exit 1; }
n435=$(lsusb | grep -c "8086:0b07")
[ "$n435" -ge 2 ] || { echo "D435 가 $n435 대만 보임 (lsusb 8086:0b07) — 두 대를 USB 3 포트에 꽂을 것"; exit 1; }
source $H/pc/env_pc.sh   # ROS setup.bash 는 set -u 와 충돌하므로 먼저 부름
set -u
mkdir -p $D
echo "== 팀 dev 빌드" | tee $D/run.log
git -C ~/git/Lv2_Cartlidar_Assignment_lab log --oneline -1 | tee -a $D/run.log
lsusb -t | grep -B1 -i "uvcvideo" | grep -o "[0-9]*M" | sort | uniq -c | sed 's/^/  USB 속도(D435 인터페이스): /' | tee -a $D/run.log
echo "== 스택" | tee -a $D/run.log
bash $H/pc/p5_pc.sh stop > /dev/null 2>&1; bash $H/jetson/bag_kill.sh > /dev/null
bash $H/pc/p5_pc.sh 2>&1 | tail -4 | tee -a $D/run.log
echo "== 기울기 (기준 |앞뒤| · |좌우| ≤ ${TILT_MAX:-2}°)" | tee -a $D/run.log
python3 $H/jetson/floor_tilt.py 6 | tee $D/tilt.txt | tee -a $D/run.log
python3 - "$D/tilt.txt" "${TILT_MAX:-2}" <<'EOF' || { echo "기울기 기준 넘음 — 로봇 · 카메라를 바로잡고 다시 (스택은 떠 있음)" | tee -a $D/run.log; exit 1; }
import re, sys
s = open(sys.argv[1]).read(); m = float(sys.argv[2])
a = [float(x) for x in re.findall(r'앞뒤 ([+-][\d.]+)°.*좌우 ([+-][\d.]+)°', s)[0]]
sys.exit(0 if abs(a[0]) <= m and abs(a[1]) <= m else 1)
EOF
python3 $H/jetson/light_height.py 6 | tee -a $D/run.log
B=$H/pc/p5_lab_bag/$(basename $D)
if [ "${BAG:-1}" != 0 ]; then
  echo "== 영상 bag (앞 컬러 compressed + /perception/signal) → $B" | tee -a $D/run.log
  mkdir -p $(dirname $B); rm -rf $B
  ros2 bag record -o $B --topics /top_camera_front/color/image_raw/compressed /perception/signal > $D/bag.log 2>&1 &
  sleep 4
fi
echo '{"offset_s": 0.0, "rtt_s": 0.0, "n": 0}' > $D/clock.json   # PC 하나라 시계 차이 없음
echo "== 판 $N 개 × $S s" | tee -a $D/run.log
for k in $(seq 1 $N); do
  for m in 0 32; do ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: $m}" 2>&1 | grep -o "success=[A-Za-z]*"; done > $D/reset_$k.txt
  date +%s.%N >> $D/reset_$k.txt
  python3 $H/jetson/sig_truth.py $D/truth_$k.log $S --new
  echo "  판 $k 끝" | tee -a $D/run.log
done
if [ "${BAG:-1}" != 0 ]; then
  bash $H/jetson/bag_kill.sh | tee -a $D/run.log
  ros2 bag info $B 2>&1 | grep -E 'Duration|Count' | tee -a $D/run.log
fi
cp $H/pc/p5_out/p5.jsonl $H/pc/p5_out/perception.log $H/pc/p5_out/sensors.log $D/
echo "== P5b 순서" | tee -a $D/run.log
python3 $H/jetson/p5e_analyze.py $D $D/p5.jsonl | tee $D/p5b.txt | tee -a $D/run.log
echo "== P5a 프레임" | tee -a $D/run.log
python3 $H/jetson/p5a_frames.py $D $D/p5.jsonl | tee $D/p5a.txt | tee -a $D/run.log
[ "${BAG:-1}" != 0 ] && echo "영상 bag: $B (사람 얼굴이 찍혔을 수 있어 팀 저장소에 넣지 않음)" | tee -a $D/run.log
echo P5_LAB_DONE
