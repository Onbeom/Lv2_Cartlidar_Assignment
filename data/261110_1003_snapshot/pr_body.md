## 무엇
- perception: `/perception/snapshot`(std_msgs/String = 경로 접두어)을 받으면 팔 카메라 모드(READ_TAG · ARM_ALIGN)에서 0.5 s 뒤부터 0.5 s 간격 3 장을 `<접두어>_1..3.jpg` 로 저장. 요청 뒤 새 영상만 쓰고 3 s 안에 안 오면 경고 뒤 포기(미션은 계속). 사진 전용 콜백 그룹 + 쓰기 스레드.
- mission: 미션(/mission/start 로 새로 시작)마다 `<log_dir>/<YYMMDD_HHMM>/` 한 폴더에 `mission_log.jsonl` · `images/`(이동 전 survey_view `NN_before_r<회차>_B<박스>`, 이동 후 verify `NN_after_<L>`) · `planner/`(계획기 기록 사본) · 퍽 등록표 사본. PAUSED 재개는 같은 폴더.
- bringup: `log_dir` 인자(기본 `~/Lv2_Cartlidar_Assignment/mission_log`) → 계획기 `<log_dir>/planner`, `pucks_file` 기본 `<log_dir>/pucks_survey.yaml`.

## 왜
- 발제 Q1 기록 항목 '이동 전후 이미지'가 없었음(#172). 상시 녹화 없이 이미 켜져 있는 팔 카메라로 사건당 3 장(미션당 27 장, settle 2.0 s 안이라 미션 시간 그대로).
- 기록이 `/tmp/mission_log`(재부팅 때 지워짐)와 `~/.ros/mission_planner` 로 나뉘어 있던 것을 미션 단위 폴더 하나로 모음.

## 시험 (PC 모의)
- 단위: test_snapshot 7 · test_episode_log 7 통과, 기존 test_perception_logic 181 · test_verify_record 7 그대로 통과.
- perception 단독(component_container_mt, 가짜 팔 영상): 요청 뒤 0.52 · 1.02 · 1.52 s 에 다른 영상 3 장, 꺼진 모드는 경고만, 영상 없으면 3.5 s 뒤 포기.
- mock 미션(진짜 mission · puck_survey · 계획기 · perception_node): DONE, 27 장(다시 보기 1 회면 30 장), 계획기 4 파일 · 등록표 사본. bringup 구성 계산 4 가지 예상대로.

## 한계
- 실물 팔 카메라 · Jetson 저장 시간 · CPU 미확인. 카메라 거꾸로 장착 회전은 넣지 않음(실물 보고 정함).
- `log_dir` 기본값은 Jetson 저장소 경로(DEPLOY.md) 기준, PC 시험은 `log_dir:=` 를 따로 줄 것. `submission/midterm/nodes_topics_states.md` 기록 표는 Issue 로 따로 요청.
