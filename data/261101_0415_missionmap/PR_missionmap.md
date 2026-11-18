## 무엇을
- `perception/config/puck_survey.yaml` 의 `world_frame` 을 `map` → `mission_map` 으로 바꿨습니다. perception_survey README 에 한 줄 적었습니다. 노드 기본값(`map`)은 그대로입니다.
- 이 브랜치는 `puck_survey.yaml` 을 만든 #24(fix/perception-survey-launch) 위에 있습니다. #24 가 먼저 들어가면 이 PR 에는 이 변경만 남습니다.

## 왜
SLAM 구성의 TF 는 `mission_map`(발제 좌표) →(정적) `map`(Cartographer 지도) → `odom` → `base_link` 입니다. 지도 원점은 지도를 만들 때마다 달라서(현재 x 0.729 · y 0.960 · yaw 3.14), 측량이 `map` 으로 받으면 발제 좌표로 고정된 칸(BOX_CENTERS)과 맞지 않아 모든 관측이 칸 밖으로 버려집니다.

## 시험
- 단위 시험: 이 브랜치 113/113, 열린 인지 PR · #32 · #33 · TFmini 끊김 수정과 합친 트리 + 이 변경 162/162(PC). 열린 PR 11개 head 와 병합 충돌 없음.
- B0′ 판정 렌더 재생(모의)에 SLAM 과 같은 꼴의 TF 를 넣음(mission_map → map = 현재 지도 값 x 0.729 · y 0.960 · yaw 3.143774, map → odom = 그 역).
  - 옛 설정(`map`): 2회 모두 실패, 관측이 모두 칸 밖(65 · 68개).
  - 새 설정(`mission_map`): 24/24회 · 288/288칸, 틀린 완료 0, 중심 오차 최대 5.35 mm(YOLO 엔진 주 검출기, 고치기 전 항등 TF 결과와 같음).

## 한계
모의 재생(SLAM 오차 없는 TF)으로만 확인했습니다. 실시간 SLAM 장면에는 컬러 영상과 퍽이 없어 붙여 보지 않았습니다. Nav2 · mission 이 같은 `mission_map` 을 쓰는지는 그쪽 설정에 달려 있습니다.
