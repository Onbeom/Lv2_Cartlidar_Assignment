#!/usr/bin/env bash
# 위치 추정 C++ 판 × GPU 왜곡 보정 측정 (11-14 검토 승인 기준, README '다음 측정 기준'). 덧씌우기 pws3, 팔 30 Hz, 계획기 끔, 5 분씩 8 판.
# 순서: 1 바퀴 GPU 켬(C++ → Python) · GPU 끔(C++ → Python), 2 바퀴는 거꾸로 — 시간에 따른 변화가 한쪽에 몰리지 않게.
# 판마다 p7a.sh 가 다른 bringup 이 있으면 재지 않고 끝냄. 끝나면 'P7LOC 끝' 과 종소리.
# 11-14 10:05: 팔 보드를 다시 꽂은 뒤 포트가 ACM0 · ACM2 가 되어 기본 후보(ACM0,ACM1)로는 하드웨어가 안 뜸 → 지금 있는 ACM 포트를 후보로 넘김(역할은 응답으로 판별)
PORTS=${PORTS:-$(ls /dev/ttyACM* 2>/dev/null | paste -sd,)}
echo "OpenCR 후보 포트 $PORTS"
ORDER="1:true 1:false 0:true 0:false 0:false 0:true 1:false 1:true"
for gc in $ORDER; do
  g=${gc%%:*}; c=${gc##*:}
  echo "== $(date +%H:%M:%S) GPU $g · loc_cpp $c"
  sleep 20
  P7_OVERLAY=~/real_drive/pws3 PERCEPTION_PROFILE=1 PERCEPTION_UNDISTORT_GPU=$g bash ~/real_drive/p7a.sh 300 use_planner:=false loc_cpp:=$c ports:=$PORTS
done
echo "P7LOC 끝 $(date +%H:%M:%S)"; printf '\a'
