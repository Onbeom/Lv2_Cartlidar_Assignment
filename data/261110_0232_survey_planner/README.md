# 261110_0232 survey → planner 한 번 실행 (할 일 2번)

빌드는 `~/git/Lv2_Cartlidar_Assignment_verify`(origin/dev e7cdde7 + PR #174) 에서 이 폴더 `install/` 로. survey · planner 코드는 dev 와 같음.
진짜: mission_node(plan, survey 켬), perception_survey `puck_survey`, mission_planner_node(`pucks_file` = mission `survey.pucks_out` 같은 경로, bringup 과 같은 연결).
가짜: `mock_robot.py`(주행 · 팔 · set_mode · check_dice), `survey_pucks_mock.py`(survey_view 동안 `truth.yaml` 배치의 퍽을 /perception/pucks 로).
`truth.yaml` 은 예제(problem_q1_example)와 ID · 모양이 다른 배치라, 계획이 측량 결과로 만들어졌는지 구분된다.

| 시험 | 종류 | 예상 (값의 뜻 · 예상과 이유) | 결과 |
|---|---|---|---|
| A 한 번에 읽힘 | 모의 | survey_finish 성공 1 번, pucks_survey.yaml 12 줄 = truth.yaml(id · 색 · 모양 · box · side), 계획기 allocation.json 의 퍽 = truth, 미션 DONE. survey_start 가 옛 파일을 지우고 계획기는 plan 때 파일을 읽으므로 | survey_finish 성공 1 번, pucks_survey.yaml = truth.yaml 12/12 일치, allocation.json 이 truth 기준(예: G 그룹 id 10 · 8 = truth 의 초록 원기둥, 예제에서는 빨강), DONE 65.8 s (`mock_A/`) |
| B 첫 방문 태그 못 읽음 (박스 3) | 모의 | 첫 finish 실패(3:-1 id 모름) → revisit 1 번(박스 3 만, TAG_VIEW · 0.20) → 두 번째 finish 성공, 이후 A 와 같음, DONE | 첫 finish 실패(3:-1 id · 색 · 모양 모름 + id_set_incomplete) → revisit 3:-1 만 → `goto · approach 0.20 · survey_view 3 TAG_VIEW` 추가 → 두 번째 finish 성공, 등록표 truth 와 12/12 일치, DONE 67.9 s (`mock_B/`) |

## 알게 된 것
- **코드 수정 없이 이어짐**: survey_start(옛 등록표 지움) → 박스 6 곳 survey_view → finish → mission 이 완료 JSON 을 `pucks_out` 등록표로 → plan 때 계획기가 같은 경로를 읽음. bringup.launch.py 의 `survey.pucks_out = pucks_file` 연결과 같다.
- 시험 스크립트 첫 실행은 plan 이 "확정된 점멸 순서 없음"으로 실패: 계획기가 지도를 읽는 약 12 s 동안 mock observe 가 끝나 신호를 못 받음(계획기 신호 구독은 volatile, 늦게 뜨면 못 받음). 계획기 ready 뒤 시작하게 스크립트를 고쳐 해결. 실물 bringup 은 계획기가 미션 시작 전에 떠 있어 해당 없음. 다만 미션 도중 계획기가 다시 뜨면 같은 실패가 난다.
- 태그를 못 읽은 팔 카메라 관측은 색 · 모양도 버려짐(태그 없는 관측의 점수 문턱 0.6 > 모의 0.5). 그래서 B 에서 3:-1 은 id · 색 · 모양 모두 모름.
- 실물에서는 survey 자세 BOX_VIEW 가 퍽을 못 봄(#154) → 할 일 5번 전에는 실물 survey 가 막힌다.
- `on_unconfirmed: registered` 는 계획기 pucks_file 이 pucks_out 과 다른 경로여야 동작하는데, bringup 은 두 값을 같은 경로로 묶음 → bringup 으로는 registered 를 쓸 수 없음(mission_plan.yaml 주석에 이미 적힘).
