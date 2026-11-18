#!/usr/bin/env bash
# 라이다 · TFmini 포트 가리기 (Jetson, 터미널에서): ttyUSB0~2 를 차례로 라이다 드라이버로 켜고(포트마다 60 s),
#   1 s 마다 '15 cm 안 점 수' 를 보여 준다. 포트마다: 앞 라이다를 손바닥으로 바짝 10 초 가리고 → 떼고 → 뒤 라이다를 10 초 가림.
#   수가 확 늘 때 가린 라이다가 그 포트. /scan_front 가 안 나오는 포트는 라이다가 아님(TFmini 후보). 결과 ~/real_drive/lidar_find.log
source ~/real_drive/env.sh
while pgrep -f "[l]idar_spin.sh" >/dev/null; do sleep 1; done
for P in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2; do
  echo; echo "########## $P — 켜는 중 (8 초 뒤 시작). 앞 라이다 10 초 가림 → 떼기 → 뒤 라이다 10 초 가림 ##########"
  bash ~/real_drive/lidar_spin.sh $P 60 > /dev/null 2>&1 &
  sleep 9
  python3 ~/real_drive/lidar_near.py 48 | sed "s#^#$(basename $P) #"
  wait
done 2>&1 | tee ~/real_drive/lidar_find.log
echo "끝 — 결과 ~/real_drive/lidar_find.log"
