**Arducam 1280×720 보정 결과 (실물, 2026-11-09)**

ChArUco 10×7 판을 68 장 찍어 OpenCV `calibrateCamera` 로 구했습니다. 화면 끝까지 덮도록 따로 찍었습니다. 카메라는 PC 에 USB 로 연결했고, 해상도 · 형식(MJPG 1280×720)은 로봇과 같습니다.

| 항목 | 기준 | 결과 |
|---|---|---|
| 재투영 RMS (맡겨 둔 장도) | ≤ 1 px | 0.203 (맡겨 둔 장 0.207) px |
| 화면 덮음 (8×5 칸) | ≥ 85 % | 98 % |
| fx 표준편차 | ≤ 1 % | 0.11 % |
| 15° 이상 기운 장 | ≥ 25 % | 57 % |
| 화면 끝 100 px 띠 흔들림 (다시 뽑아 30 회 맞춤, p95 최대) | ≤ 10 px | 2.3 ~ 3.4 px |

- **fx 865 px** 입니다. 시뮬레이션의 팔 카메라 가정(약 424 px, 화각 113°)과 많이 다릅니다.
- 왜곡이 커서 rational_polynomial(계수 8 개)이 맡겨 둔 장에서 plumb_bob 보다 나아 이것을 골랐습니다. perception 은 계수 8 개를 그대로 받습니다.
- perception 이 펼친 영상은 fx 약 602 px (가로 화각 약 93°) 입니다.

```yaml
image_width: 1280
image_height: 720
camera_name: arm_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [865.4326572, 0, 694.8757499, 0, 865.561246, 359.636803, 0, 0, 1]
distortion_model: rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 8
  data: [7.015057401, 21.58684794, -0.0003022761007, 0.0004858712959, 2.668479135, 7.439728992, 24.81589374, 11.50292432]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [865.4326572, 0, 694.8757499, 0, 0, 865.561246, 359.636803, 0, 0, 0, 1, 0]
```

이 파일을 저장소에 넣고 bringup 기본값으로 연결하는 PR 을 올리겠습니다. 적용 뒤 `/perception/pucks` 확인은 실물 시험 때 하겠습니다.
