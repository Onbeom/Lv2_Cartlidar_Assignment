# #193 등록표에 pose 쓰기 (mission_node)

- 작업 트리 `~/git/Lv2_Cartlidar_Assignment_surveypose`, 브랜치 `fix/survey-pose-pucks`(origin/dev 7cb49b0, PR #186 병합 뒤).
- 고친 것: `writePucksFromSurvey`(LCY 11-03 작성)의 변환 부분을 헤더 `include/mission/survey_pucks.hpp` 로 옮기고 pose 를 더함.
  - pose standing | lying 이면 `pose: …` 를 씀.
  - 없거나 null 이면 쓰지 않음(계획기는 standing 으로 읽음).
  - 그 밖의 값이면 오류.
  - 나머지(12 칸, id · color · shape 필수, cuboid → box)는 그대로.
- 측량 완료 파일에는 이미 pose 가 있음. `perception_survey/node.py:114` 가 축 기울기 20° 로 정하고, `core.py:112` 는 pose 가 정해지지 않으면 complete 가 안 됨.
- 빌드: `mission` 하나만, `261110_1003_snapshot/install` 위에 얹음(RL 학습 중이라 `nice` · 병렬 4).

## 시험 (PC, 빌드 · 단위 · 계산. 모의 미션 아님)

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| U1 | `mission` 빌드 | 경고 · 오류 없이 빌드(바뀐 것은 헤더 하나 · 함수 하나) | 통과. 29.3 s, 컴파일 경고 0(경고 1 줄은 colcon 의 '아래 설치본에 이미 있음' 안내). `build.txt` |
| U2 | 새 gtest `test_survey_pucks` | 3 개 통과: pose 옮김(lying 1 · standing 11, cuboid → box, 다시 읽으면 12 개), 없음 · null 은 키 생략, 틀린 pose · 11 칸 · id 없음은 실패 | 통과(3 tests) |
| U3 | 기존 gtest(`test_verify_record`, `test_episode_log`) | 바뀐 곳과 무관해 모두 통과 | 통과(7 · 7) |
| U4 | 계획기 연결 | 헤더로 만든 등록표(B4 +X 직육면체 누움)를 dev 계획기는 pose 를 무시하고 받아서, pose 를 뺀 등록표와 문장이 같음. `fix/lie-pick-order` 계획기는 그 퍽을 lying 으로 읽어 `lie_skill` 을 주면 그 퍽만 누운 퍽 스킬 | 통과. 등록표 pose: 7 만 lying. dev(7cb49b0) 계획기 문장이 pose 있음 · 없음 같음(per_location 105 줄 · single_load 99 줄). lie 브랜치는 두 모드 모두 누운 퍽 스킬 1 줄(처음 `lie_regrasp_tilt`, 11-11 문장 순서 바꾼 뒤 `lie_upright` 로 다시 돌려 같음). `u4.txt` · `u4.py` · `dump_pucks.cpp`(헤더를 그대로 씀) |
