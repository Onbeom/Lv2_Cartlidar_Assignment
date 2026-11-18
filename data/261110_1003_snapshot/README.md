# 261110_1003 이동 전후 사진 저장 (4번)

작업 트리 `~/git/Lv2_Cartlidar_Assignment_snapshot`, 브랜치 `fix/mission-snapshot`.
아직 병합 전인 PR #174 · #175 · #171 · #176 를 로컬에서 합친 커밋(b3ccaa7, 'tmp base' 병합 4 개) 위에서 작업했다.
PR 들이 병합된 뒤 `git rebase --onto origin/dev b3ccaa7 fix/mission-snapshot` 로 옮겨서 올린다.

빌드 · 시험 결과물은 이 폴더(`build/ install/ log/`). `arm_cam_mock.py` · `survey_pucks_mock.py` · `truth.yaml` · `run_*.sh` · `bringup_check.py` 는 확인용이라 팀 저장소에 넣지 않는다.

## 한 것

- perception: `/perception/snapshot`(String = 경로 접두어) → 팔 카메라 모드(READ_TAG · ARM_ALIGN)일 때 0.5 s 뒤부터 0.5 s 간격 3 장 `<접두어>_1..3.jpg`(품질 90). 장마다 요청 뒤 새로 받은 영상만, 3 s 안에 안 오면 남은 장 포기(경고). 쓰기 스레드 하나, 잠금 안에서는 포인터 복사만. 이름이 있으면 `_2`. 파라미터 `snapshot.count · delay_s · interval_s · max_wait_s · jpeg_quality`. 새 헤더 `snapshot.hpp` + 시험 `test_snapshot`.
- mission: 에피소드마다 `<log_dir>/<YYMMDD_HHMM>/`(벽시계, 있으면 `_2`), beginMission 맨 앞에서 정함. `mission_log.jsonl` 은 그 폴더에. survey_view · verify 의 setModeThen 람다에서 `images/<NN>_before_r<회차>_B<박스>` · `images/<NN>_after_<L>` 발행 + `snapshot` 줄. plan 성공 뒤 `<log_dir>/planner` 파일을 `planner/` 로 복사, survey_finish 성공 뒤 등록표 · 완료 파일 사본. `log_dir` · `survey.pucks_out` 앞 `~` → HOME. `survey.pucks_out` 빈 값 = `<log_dir>/pucks_survey.yaml`. 새 헤더 `episode_log.hpp` + 시험 `test_episode_log`.
- bringup: `log_dir` 인자(기본 `~/Lv2_Cartlidar_Assignment/mission_log`, expanduser) → mission `log_dir` · 계획기 `<log_dir>/planner`, `pucks_file` 기본 빈 값 = `<log_dir>/pucks_survey.yaml`.
- mission_plan.yaml `log_dir` · `pucks_out`, gazebo_mission.sh 계획기 `log_dir` → `$OUT/log/planner`.

## 시험 (모두 PC 모의. 실물 · Gazebo 아님)

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| U1 | `test_snapshot` | 7 개 통과 (일정 0.5 · 1.0 · 1.5 s, 새 영상만, 3 s 뒤 포기, 덮어쓰기 없음, 쓰기 순서) || 통과 7/7 (`unit_tests.txt`) |
| U2 | `test_episode_log` | 7 개 통과 (`~` 풀기, YYMMDD_HHMM, `_2` · `_3`, 접두어 두 자리, 사본) || 통과 7/7 |
| U3 | 회귀: `test_perception_logic` · `test_verify_record` | 181 · 7 개 그대로 통과 (기존 동작 안 바뀜) || 통과 181 · 7 |
| P1 | perception 단독, READ_TAG + 영상 15 Hz, 요청 1 번 | `01_test_1..3.jpg` 3 장, 로그 '사진 n/3 저장' 3 줄, 세 장의 프레임 번호가 다름 || 통과. 요청 뒤 0.52 · 1.02 · 1.52 s 에 3 장, 프레임 82 · 89 · 97 (`p_P1/`). ※ 첫 시도는 아래 '시험 중 고친 것' 참고 |
| P2 | 팔 카메라 꺼진 모드에서 요청 (`--no_mode`) | 경고 1 줄 '꺼진 모드', 파일 0 || 통과. 경고 1 줄, 파일 0 |
| P3 | READ_TAG 인데 영상 없음 (`--hz 0`) | 약 3.5 s 뒤 '1/3 장부터 포기' 경고, 파일 0 || 통과. 요청 뒤 3.50 s 에 '1/3 장부터 포기', 파일 0 |
| M1 | mock 미션 (측량 켬, 다시 보기 없음) | DONE. `mission_log/` 아래 에피소드 폴더 1 개, `images/` 27 장 (before_r1_B1~B6 × 3 + after_L1~L3 × 3, 번호 01~09), jsonl `snapshot` 9 줄, `planner/` 4 파일, `pucks_survey.yaml` · `puck_survey.json` 사본 || 통과. DONE, `mock_M1/mission_log/261110_1014/` 하나, 27 장 01~09 (B1~B6 · L2 · L3 · L1 순), snapshot 9 줄, planner 4 파일 (copy 줄 ok), 등록표 · `puck_survey.json` 사본, perception 포기 · 버림 0 |
| M2 | mock 미션 `--unread_box 3` (다시 보기 1 회) | DONE. 30 장 (before_r2_B3 × 3 추가, 번호 01~10), 이름 겹침 없음 || 통과. DONE, 30 장, `07_before_r2_B3` 추가 (revisit ok), 포기 · 버림 0 |
| B1 | bringup 구성 계산: 기본 | mission `log_dir` = `$HOME/Lv2_Cartlidar_Assignment/mission_log`, `survey.pucks_out` = 그 아래 `pucks_survey.yaml`, 계획기 `log_dir` = 그 아래 `planner`, `pucks_file` 같음 || 통과 (`bringup_check.txt`) |
| B2 | `log_dir:=/tmp/x` | `/tmp/x` · `/tmp/x/pucks_survey.yaml` · `/tmp/x/planner` || 통과 |
| B3 | `pucks_file:=/tmp/p.yaml` | mission · 계획기 둘 다 `/tmp/p.yaml`, log_dir 기본 || 통과 |
| B4 | `mission_source:=tag` | mission 에 log_dir 안 넘김 (기존과 같음), 계획기 없음 || 통과. 덧붙여 plan 에서 `log_dir:= pucks_file:=` 둘 다 비우면 launch 오류로 멈춤 |

## 시험 중 고친 것

- 첫 P1 (단일 스레드 `ros2 run`): 2 번째 장이 1.0 s 가 아니라 1.87 s. 저장된 영상 글자로 보니 1 번째 장 뒤 1.5 s 동안 새 영상이 없었음.
  사진 일정이 처리 주기와 같은 콜백 그룹(ctl_cbg_)이라 처리가 늦으면 사진도 밀림 → 사진 전용 콜백 그룹(snap_cbg_)으로 떼고,
  팔 카메라 모드는 atomic 으로 읽게 고침. 시험은 실물처럼 `component_container_mt` (`perception_mt.launch.py`).
- 두 번째 P1: 3 s 안에 새 영상이 안 와 포기. 모의 영상(640×360 bgr8, 691 KB)을 best effort 로 다른 프로세스에 보내 조각 손실로 버려짐
  (perception 로그에 영상이 몇 초 간격으로만 도착). 시험에서만 perception `image_qos: reliable` 로 덮어씀. 실물 팔 카메라 전송 방식은 이 작업 범위 밖.

## 안 한 것 · 한계

- 실물 팔 카메라 · Jetson 저장 시간 · CPU, 거꾸로 장착 영상 회전(`snapshot.rotate_180` 은 두지 않음 — 실물 보고 정함).
- 모의에서 perception 모드는 시험 보조가 READ_TAG 로 계속 켜 둠(mock_robot 이 set_mode 를 받으므로). 미션 모드를 따라 켜지고 꺼지는 경로는 P2(꺼진 모드)와 코드 읽기로만 확인.
- PAUSED 재개 · 정지 뒤 새 시작의 폴더 경계는 코드 구조(재개는 enterStep, 새 시작만 beginMission)와 U2 로 확인. 실행 시험 없음.
