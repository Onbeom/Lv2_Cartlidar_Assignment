## 무엇을
`perception/udev/99-sensors.rules` 라이다 두 줄을 허브 칸 + 시리얼 0001 로 보게 바꿈: `ID_PATH` 의 Jetson 쪽 포트 자리를 `?` 로 (`…usb-0:2.?.2:1.0` = 앞, `…:2.?.1:1.0` = 뒤). 주석에 허브 칸 배정(2 번 칸 = 앞, 1 번 칸 = 뒤)을 적음.

## 왜
11-15 실물에서 허브를 Jetson 포트 2.3 → 2.2 → 2.1 로 옮길 때마다 `/dev/lidar_front` · `lidar_rear` 링크가 사라져 bringup 전에 규칙을 다시 고쳐야 했음. 라이다 변환기 두 개는 시리얼이 같아(0001) 시리얼로는 가를 수 없음.

## 시험
- Jetson 실물 (11-15 22:0x): 이 규칙을 /etc 에 넣고 허브를 2.1 에 꽂은 채 링크 확인 (`lidar_front → ttyUSB1` = 2.1.2, `lidar_rear → ttyUSB0` = 2.1.1), bringup 뒤 앞 상자가 `/scan_merged` 정면(+7.3°)에 찍힘. 같은 칸 배정으로 19:53 에 앞 · 뒤 모두 확인.
- 시리얼 0001 조건을 넣은 이 판을 /etc 에 넣고 다시 확인 (11-16 00:3x, 허브 Jetson 2.1, TFmini 허브 6 번 칸): `lidar_front → ttyUSB1` (2.1.2) · `lidar_rear → ttyUSB0` (2.1.1) · `tfmini → ttyUSB2` (2.1.6), 셋 다 다른 장치.
- TFmini(시리얼) · Arducam 줄은 그대로.

## 한계
- 허브 칸을 바꿔 꽂으면 여전히 바뀜 (칸 표시 필요). Jetson 에 직접 꽂으면 안 잡힘.
- TFmini 는 시리얼로 잡으므로 칸과 무관. 라이다 줄은 시리얼 0001 도 보므로 TFmini 를 라이다 칸에 꽂아도 라이다 이름을 가로채지 않음.
