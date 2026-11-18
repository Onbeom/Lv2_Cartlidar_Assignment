## 무엇
- `mission_node` 의 `writePucksFromSurvey` 가 측량 완료 파일의 `pose` 를 퍽 등록표에 옮겨 씁니다(#193).
  - 값은 `standing` | `lying` 입니다. 없거나 null 이면 쓰지 않고, 그 밖의 값이면 실패합니다.
- 변환 부분을 ROS 없이 시험할 수 있게 헤더 `include/mission/survey_pucks.hpp` 로 옮겼습니다. 12 칸 검사, id · color · shape 필수, cuboid → box 는 그대로입니다.

## 왜
- #193: 계획기가 누운 직육면체에 누운 퍽 스킬을 부르려면 등록표에 pose 가 있어야 합니다.
- 측량(`perception_survey`)은 이미 칸마다 pose 를 정합니다. 그런데 등록표로 옮길 때 빠지고 있었습니다.

## 시험 (PC, 빌드 · 단위)
- `mission` 빌드 시 컴파일 경고 0 입니다.
- 새 gtest `test_survey_pucks` 3 개, 기존 `test_verify_record` 7 개 · `test_episode_log` 7 개 모두 통과했습니다.
- 헤더로 만든 등록표(B4 +X 누움)를 dev 계획기에 넣으면 pose 를 무시합니다. pose 가 없을 때와 문장 목록이 같았습니다(per_location 105 줄 · single_load 99 줄).

## 한계
- 지금 dev 계획기는 pose 를 읽지 않습니다. 계획기 쪽(pose 읽기 · 누운 퍽 스킬 문장)은 따로 PR 로 올립니다.
- 모의 미션 · 실물에서는 확인하지 않았습니다. 고친 함수는 LCY 님 코드라 리뷰를 부탁드립니다.
