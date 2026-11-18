제목: perception: udev 규칙에 TFmini · 라이다 실제 값 넣기

## 무엇
- `perception/udev/99-sensors.rules` 의 자리표시(TFMINI_SERIAL · LIDAR_FRONT_SERIAL · LIDAR_REAR_SERIAL)를 실물 값으로 바꿈.
- TFmini 변환기(CP2102N)는 고유 시리얼로, 라이다 두 대는 USB 경로(ID_PATH, 앞 2.3.5 · 뒤 2.3.1)로 잡음.

## 왜
- 자리표시 그대로라 Jetson 에 /dev/lidar_front · /dev/lidar_rear · /dev/tfmini 가 안 생겨 bringup 의 라이다 · TFmini 노드가 포트를 못 엶.
- 라이다 두 변환기(CP2102)는 시리얼이 둘 다 0001 이라 시리얼로는 가를 수 없음.

## 시험
- PC `udevadm verify` 통과. Jetson `udevadm info` 로 세 장치의 시리얼 · ID_PATH 확인, 앞 · 뒤는 포트마다 라이다를 하나씩 띄우고 앞 · 뒤를 번갈아 가려, 가렸을 때 스캔이 바뀌는 쪽으로 정함(실물 11-12).
- TFmini 는 남은 포트(2.3.2, CP2102N)이고, 버전 요청에 응답(5a 07 01 …)이 와서 TFmini 임을 확인함.
- Jetson 설치 뒤(11-13): /dev/lidar_front → ttyUSB2(2.3.5), /dev/lidar_rear → ttyUSB0(2.3.1), /dev/tfmini → ttyUSB1(CP2102N), /dev/arm_cam 그대로.

## 한계
- 라이다 USB 선을 허브의 다른 포트에 꽂으면 경로가 바뀌어 규칙을 다시 고쳐야 함.
- TFmini 는 이름만 생기고, 지금 측정 프레임이 거의 안 나옴(선 · 전원 확인 전) — 이 PR 과 별개.
