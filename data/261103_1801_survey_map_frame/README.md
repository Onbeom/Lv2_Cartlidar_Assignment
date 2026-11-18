# 측량 world_frame → map (localization 패키지 꼴) · 2026-11-03 (모의, PC)

왜: YOLO 1103 1이 11-03에 알려 왔다. 새 팀 패키지 `localization`(feat/localization, 커밋 전)은 발제 좌표를 `map`으로 내고, Cartographer 지도는 `map → carto_map`으로 둔다(`localization.launch.py`, `map_odom_relay.py`).
지금 dev의 `puck_survey.yaml`은 `world_frame: mission_map`이다. 이 frame을 내는 노드가 없으므로, 측량은 관측을 하나도 넣지 못한다.

바꾼 것: 브랜치 `fix/perception-survey-map-frame`(작업 트리 `~/git/Lv2_Cartlidar_Assignment_mapframe`, dev c27d100 위).
- `puck_survey.yaml`: `world_frame: map`
- `perception_survey/README.md`: 한 줄

시험: F1(D)(`261103_1546_yolo_D_f1`)과 같은 렌더 · D 엔진 · 기준을 쓴다. `--mission-map`은 주지 않는다. 그러면 재생기가 `map → odom`을 항등으로 내는데, 이것이 발제 좌표 = map인 새 꼴이다.

| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| 빌드 · survey 시험 | 통과 | 통과. 설정 · 문서만 바뀜 | **통과**: survey 21개 시험 실패 0 |
| F1 24회 · 칸 | 24/24 · 288/288 | 24/24 · 288/288. 좌표는 F1(D)과 같은 월드라 같은 결과 | **24/24 · 288/288**. 기동 12/12 'tensorrt … 클래스 4' · FATAL 0 |
| 틀린 완료 · 중심 최대 · 집게 헛 퍽 | 0 · ≤ 10 mm · 0 | 0 · 5.33 mm 근처 · 0 | **0 · 5.33 mm · 0**(ID · 색 · 모양 · 자세 틀림 0, D435 퍽 642) |
| 대조: 옛 설정(mission_map)을 새 꼴에(seed 101 T3000) | 기준 없음(문제 확인) | 0/2. 관측이 TF를 못 찾아 들어가지 않음 | **0/2 · 0/24 칸**. 모든 칸 관측 0개(n_obs 0), 틀린 완료 0 |

결과 `f1/replay_*/summary.json`, 대조 `f1/control_mission_map_101_T3000/`(2026-11-03). **통과(모의).** localization 패키지 PR과 같이, 또는 그 뒤에 합친다(그 전 dev에는 map이 발제 좌표인 구성이 없다).
