## 요청

모의(Isaac Lab · Isaac Sim 브리지 · Gazebo)의 팔 카메라(Arducam) 화각을 실물 보정값에 맞춰 주세요. 지금은 세 곳 모두 가정값이고, 실물보다 훨씬 넓게 봅니다.

## 왜

- 11-09 실물 보정(1280×720, 68 장, 재투영 RMS 0.203 px)에서 fx 865.4 · fy 865.6 · cx 694.9 · cy 359.6 px 가 나왔습니다. 바늘구멍으로 바꾸면 가로 화각 약 73° 입니다. 왜곡이 커서 `rational_polynomial` 계수 8 개를 씁니다. 보정 파일은 #143 PR 로 `perception/config/arm_camera_1280x720.yaml` 에 들어갈 예정입니다.
- 모의 값은 다음과 같습니다.

| 위치 | 지금 값 | 쓰는 곳 |
|---|---|---|
| `urdf_and_usd/waffle_4wd_description/config/sensors.yaml` `hfov_rad` | 1.9721 rad(가로 113°, 대각 120° 에서 바꾼 가정) | Isaac Lab `isaac/waffle_4wd_isaac/sensors_cfg.py` |
| `urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf.xacro` `arm_camera` `horizontal_fov` | 1.9721 rad | Gazebo |
| `tools/isaac_sim/waffle_bridge.py` `ARM_RES, ARM_HFOV` | 640×480, 105°(근사, ★ 실물 보정값으로) | Isaac Sim 브리지 |

- 모의 화면이 실물보다 넓으면, 모의에서 퍽이 보이던 자세(예: target_look, 정렬 자세)가 실물에서는 화면 밖일 수 있습니다. 화면 위 · 아래 끝은 광축에서 약 24° 입니다(보정 모형 기준).

## 할 일

- [ ] 세 곳의 팔 카메라를 1280×720, fx 865.4 px(가로 약 73°)로 맞추기. 왜곡은 넣을 수 있는 곳만 넣고, 못 넣으면 그렇다고 적기
- [ ] 바꾼 뒤 모의에서 target_look · 정렬 자세에 퍽이 화면 안에 들어오는지 확인
- [ ] 결과를 이 Issue 댓글로 남기기

## 참고

- 보정은 Arducam 을 팔에 단 채 PC 에 USB 로 연결하고, 보정판을 옮겨 가며 찍었습니다. 위 값은 렌즈 내부값이며, 카메라 장착 위치 · 각도(외부값) 보정은 아직 안 했습니다.
- 화면 좌우 끝 광선은 왼 51.2° · 오른 41.4° 로, 주점이 가운데에서 오른쪽으로 약 55 px 치우쳐 있습니다.
