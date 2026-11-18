`fix/mission-snapshot` PR 에서 이동 전후 이미지 저장과 미션 기록 폴더가 바뀌어, 중간 제출 문서 `submission/midterm/nodes_topics_states.md` 가 지금 코드와 다릅니다. 문서 작성자께 고쳐 주시기를 요청합니다(제출 묶음이라 직접 고치지 않음).

## 고칠 곳
- 2 절 그래프: `MIS --> PER` 에 `/perception/snapshot` (std_msgs/String, 사진 저장 경로 접두어) 추가.
- 4 절 기록 표 (147 줄 부근):
  - 위치: 미션마다 `<log_dir>/<YYMMDD_HHMM>/` 한 폴더 (bringup 기본 `~/Lv2_Cartlidar_Assignment/mission_log`). `mission_log.jsonl` 은 그 안, `snapshot` · `copy` 줄이 늘어남.
  - 계획기 4 파일: 계획기 `log_dir` = `<log_dir>/planner`, plan 성공 뒤 에피소드 폴더 `planner/` 로 사본.
  - `pucks_survey.yaml`: `<log_dir>/pucks_survey.yaml`, survey 성공 뒤 에피소드 폴더에 사본.
  - '이동 전후 이미지 · 영상 — 없음 (미구현)' → `images/` 팔 카메라 사건당 3 장: 이동 전 `NN_before_r<회차>_B<박스>_<k>.jpg` (survey_view), 이동 후 `NN_after_<L>_<k>.jpg` (verify). 전체 영상은 따로.

자세한 동작은 `mission_planner/README.md` 의 '기록' 문단과 `mission_node.cpp` 머리말 '기록' 을 보면 됩니다.
