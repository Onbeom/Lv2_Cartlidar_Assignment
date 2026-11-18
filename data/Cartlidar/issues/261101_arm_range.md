## 요청 (인지 파트 → control)

drive_node(정밀 정렬 FINE)와 manipulator_node(pick 목표 ① · 미세 보정)는 `/arm_range`(sensor_msgs/Range, `names::ARM_RANGE`)를 받는데, dev(d04599b)에는 이 토픽을 내는 노드가 없습니다.

- 실물 구성: 팔 끝에는 Arducam 만 있고 거리 센서가 없습니다. TFmini 는 1층 정면 바닥 중앙에 달린 앞쪽 거리 센서이고, 인지의 `tfmini_node` 는 `/tfmini/range` 로 냅니다(TFmini 비상 정지 `obstacle_stop` 입력, 10-22 변경).
- 그래서 지금 코드로는 FINE 정렬이 `lost_s` 뒤 "거리 무효 (/arm_range)"로 실패하고, pick 은 ① 없이 ② object_pose 로만 갑니다.

정할 것: 팔 정렬 전후 거리를 무엇으로 볼지(D435 깊이로 본 퍽 위치 `object_pose`, TFmini `/tfmini/range` 를 `/arm_range` 로 쓰기 — 단 바닥 정면 고정이라 팔 축 거리 아님, 또는 다른 센서). 인지가 내야 할 값이 있으면 알려 주세요.
