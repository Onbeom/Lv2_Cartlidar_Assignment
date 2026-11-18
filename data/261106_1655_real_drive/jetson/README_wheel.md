# 바퀴 보드 시험 (보드가 꽂힌 컴퓨터에서)

바퀴를 바닥에서 띄우고 시작한다. 주행 펌웨어는 실물에서 처음 돈다.

1. 펌웨어 업로드 (x86-64 PC, 저장소 dev): `workspace/ros2_ws/src/opencr_firmware/tools/build_firmware.sh wheel --upload <포트>`
2. 도구 받기 (PC): `rsync -av cart11@cart11.local:~/real_drive/ ~/real_drive/`
3. 보드 확인: `python3 ~/real_drive/team/workspace/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py monitor --port <포트> --seconds 3`
   `role=1 fw=5` 면 맞는 보드다. 이 도구는 팔용이라 끝에 "팔 보드 v5 가 아님" 이 나오는 것이 정상이다.
   `MOTOR_CONFIG` · `FAULT` 면 12 V 를 켠 채 리셋. 그래도 남으면 모터 ID(왼앞 1 · 왼뒤 3 · 오른앞 2 · 오른뒤 4, config.h) 를 확인한다.
4. 조종: `python3 ~/real_drive/robot_teleop.py --arm_port none`
   r 시작, w s a d 누르는 동안 이동, space 멈춤, e 비상 정지, q 끝. 1 단계 0.05 m/s.

판정: 네 바퀴 방향이 명령과 맞음(w 전진, a 왼쪽 회전), 키를 떼면 멈춤, e · q 로 멈춤.

보드를 Jetson 으로 옮긴 뒤에는 Jetson 에서 바퀴 · 팔을 함께 조종할 수 있다: `python3 ~/real_drive/robot_teleop.py`
