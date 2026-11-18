# 남은 실물 시험 한 번에 (11-12, 검토 '미션 파트 261111 1' 조건부 승인 반영 · 바닥 · 자동 주행 판)

- 모든 명령은 **PC 터미널**에서 친다(터미널 2). 6 단계의 팔 조작만 Jetson 대화형 ssh 창(터미널 1)에서 키로 한다.
- 각 단계 끝 줄 `RUN_DONE <단계>` 가 나오면 다음으로 간다. 실패해도 다음 단계로 가고, 마지막에 모두 가져온다.
- **바닥에서** 한다. 노트북은 바닥에 펼쳐 눕혀 화면이 천장을 보게 한다(바닥이 기울기 기준이라 눕혀야 함). 받침 상자는 없어도 된다.
- 수평계(iPhone LevelTool, 긴 쪽 = 로봇 앞뒤): 배치 A · C · E 마다 **화면 위**와 **로봇 옆 바닥**에서 위 · 아래 숫자를 모두 적는다.
- 주행은 자동이다(`drive_auto.py`, 0.05 m/s 직진만, 회전 없음, 한 번에 25 cm). 키를 누를 필요 없다. 멈추려면 그 터미널에서 Ctrl-C(바로 비상 정지 · 바퀴 토크 끔). 주행 명령은 Ctrl-C 가 Jetson 까지 가도록 `ssh -t` 로 친다.
- 팀원이 Jetson OpenCR(`meas_session.py` 등)이나 카메라를 쓰고 있으면 끝난 뒤에 한다.

## 0. 준비 (PC)
- (필수) Jetson 을 켠 뒤 공유 연결(비밀번호 직접, 이 창은 열어 둠):
  `ssh -o ControlMaster=auto -o ControlPath=~/.ssh/lv2_jetson.sock -o ControlPersist=180m -o ServerAliveInterval=15 cart11@cart11.local`
- (필수) 측정 도구 보내기:
  `scp -r -o ControlPath=~/.ssh/lv2_jetson.sock ~/git/lv2_next_module/261112_1221_front_d435_cal/jetson/. cart11@cart11.local:~/front_cal/`
- (필수) 주행 · 팔 도구 보내기:
  `cd ~/git/lv2_next_module/261106_1655_real_drive/jetson && scp -o ControlPath=~/.ssh/lv2_jetson.sock robot_teleop.py teleop_common.py arm_handeye.py charuco_util.py env.sh cart11@cart11.local:~/real_drive/`

## 1. J1 Jetson 처리 시간 (사람 일 없음, 각 3 ~ 5 분)
- (필수) 판정(카메라 3 대 + perception 노드를 스스로 띄움, 6 단계 전에): `ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh J1'`
- (선택 · 권장, 비교 기록) 빈 상태: `ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh J1idle'`

## 2. 앞 D435 요 (배치 A: 노트북을 로봇 앞 바닥에, 앞바퀴 앞 끝에서 약 30 cm, 태그 화면 전체 화면)
- 화면 긴 변을 로봇 좌우로, 화면 위 테두리를 로봇 쪽으로. 로봇 뒤로 40 cm 비어 있게(케이블 여유 포함). 팔은 접은 자세.
- (필수) 수평계 적기.
- (필수) 멈춤 확인 — 1 초 뒤로(약 5 cm) 갔다 서는지: `ssh -t -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh DT'`
  - 안 서면 Ctrl-C 하고 여기서 멈춘다.
- (필수) 찍기 + 자동 주행(약 45 s, 화면을 다시 전체 화면으로 돌릴 20 s 뒤 시작): `sleep 20; ssh -t -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh Y1'`
  - 움직임: 3 s 멈춤 → 뒤로 5 s → 3 s 멈춤 → 앞으로 5 s → 3 s 멈춤 → 한 번 더 → 멈춤(처음 자리 근처).
- (필수) 같은 배치로 한 번 더(확인용): `sleep 20; ssh -t -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh Y2'`

## 3. 실제 상자 (배치 B: 노트북 치움, 205 mm 상자 하나, 앞바퀴 앞 끝 선에서 줄자)
| 단계 | 놓는 법 |
|---|---|
| R2_none | 상자 없이 빈 바닥 |
| R2_0a | 가까운 변을 앞바퀴 선과 나란히 30 cm(양쪽 바퀴에서 같은 거리), 좌우 가운데 |
| R2_0b | 같은 방식 60 cm |
| R2_45a | 45° 돌려 가까운 꼭짓점을 가운데 30 cm, 옆 두 꼭짓점이 앞바퀴 선에서 같은 거리 |
| R2_45b | 같은 방식 55 cm |
- 놓을 때마다(필수, 이름만 바꿈): `ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh R2_none'`

## 4. 뒤 D435 (배치 C: 노트북을 로봇 뒤 바닥에, 뒷바퀴 뒤 끝에서 약 30 cm, 화면 위 테두리를 로봇 쪽으로)
- 로봇 앞으로 40 cm 비어 있게. (필수) 수평계 적기.
- (필수) 찍기 + 자동 주행(먼저 앞으로 → 뒤로, 두 번): `sleep 20; ssh -t -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh RY1'`
- (필수) 한 번 더: `sleep 20; ssh -t -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh RY2'`

## 5. 뒤 깊이 (배치 D: 노트북 치움, 로봇 뒤 바닥만 보이게)
- (필수) `ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local 'bash ~/front_cal/run.sh RD'`

## 6. 팔 카메라 손-눈 보정 + 카메라 높이 (배치 E: 노트북을 로봇 앞 바닥 35 ~ 45 cm, ChArUco 화면)
- 노트북에 `~/git/lv2_next_module/261106_1655_real_drive/jetson/charuco_10x7_5x5_100.png` 전체 화면, 화면 위 테두리를 로봇 쪽으로.
- (필수) 잴 것: 보드 칸 9 개 길이(가로 · 세로 각각), 화면 가운데의 바닥 위 높이(자), 수평계.
- (필수) 팔 카메라 · 팔 OpenCR 을 쓰는 다른 노드는 꺼 둔다.
- (필수) 터미널 1(Jetson 에 `ssh -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local` 로 들어가서):
  `cd ~/real_drive && source env.sh && python3 arm_handeye.py --out ~/real_drive/handeye/p2b_1112 --poses ~/front_cal/fk_poses.yaml --any --notes "빈 손"`
  - 손-눈 보정: 1 ~ 5 로 관절 고르고 w / s 로 움직여 보드가 보이는 자세 20 장 넘게 `c` 저장(J1 · J4 · J5 섞어 관절마다 0.6 rad 이상). 닿을 것 같으면 `space`.
  - 카메라 높이: `p` 로 BOXLOOK_PLACE → `g` 로 한 관절씩 이동 → 보드가 화면에 다 들어오게 노트북만 옮김 → `c` 3 장. BOXLOOK_TAG 도 같게. 노트북을 옮겼으면 화면 높이 · 수평계를 다시 적음. 끝나면 `p` 로 DRIVE → `g` 로 돌아감 → `q`.

## 7. 모두 가져오기 (PC)
- (필수) `scp -r -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local:~/front_cal ~/git/lv2_next_module/261112_1221_front_d435_cal/real/run_1112`
- (필수) `scp -r -o ControlPath=~/.ssh/lv2_jetson.sock cart11@cart11.local:~/real_drive/handeye/p2b_1112 ~/git/lv2_next_module/261112_1221_front_d435_cal/real/run_1112/`
- 채팅으로: 배치 A · C · E 수평계, 칸 길이(가로 · 세로), 화면 높이.

## 풀이 (나, PC — 결과가 오면 한 번에)
- 앞 요: `python3 drive_yaw.py solve real/run_1112/front_cal/Y1 real/run_1112/front_cal/Y2 --cam front --tilt <A 앞뒤 차> <A 좌우 차>`
- 뒤: `python3 drive_yaw.py solve .../RY1 .../RY2 --cam rear --tilt <C 차>` · `python3 depth_plane.py .../RD --xyz -0.2079097 -0.0250168 0.4836226 --rpy 0 0.6144098 3.141593`
- 상자: `python3 r2_box.py .../R2_none .../R2_0a .../R2_0b .../R2_45a .../R2_45b [--yaw_deg <앞 요>]`
- 손-눈: `python3 ../261106_1655_real_drive/jetson/handeye_solve.py .../p2b_1112 <xacro> <칸 m> --square_y <세로 칸 m>`
- 카메라 높이: `python3 fk_height.py .../p2b_1112 <xacro> --square <칸 m> --square_y <세로 칸 m> --screen_h <mm> --tilt <E 차>`
- J1: `real/run_1112/front_cal/J1_result.txt`, `J1idle_result.txt`
- 수평계 부호: 위 숫자 = 앞뒤, 로봇 앞을 들면 위 숫자가 줄어듦 → `--tilt` 앞뒤 = −(화면 − 바닥). 좌우 부호는 아직 확인 못 함(롤에만 영향).
