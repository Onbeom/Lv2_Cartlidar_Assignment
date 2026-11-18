# P5e (PC 에서 실행): Jetson 인지 신호등 초기화 → 연습 신호등 새 순서 + 정답 기록, 이것을 판 수만큼. 사용: bash p5e_pc.sh <결과 폴더> <판 수> <판마다 초>
D=$1; N=${2:-2}; S=${3:-110}
H=/home/pa4/git/lv2_next_module/261106_1655_real_drive
mkdir -p $D
for k in $(seq 1 $N); do
  ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'source ~/real_drive/env.sh >/dev/null 2>&1; for m in 0 32; do ros2 service call /perception/set_mode robot_interfaces/srv/SetPerceptionMode "{mode: $m}" 2>&1 | grep -o "success=[A-Za-z]*"; done; date +%s.%N' > $D/reset_$k.txt
  python3 $H/jetson/sig_truth.py $D/truth_$k.log $S --new
done
echo P5E_PC_DONE
