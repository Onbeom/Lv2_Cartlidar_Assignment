# 재현 시험 기록

목적: 만든 사람이 아닌 팀원 (또는 다른 장비) 이 같은 절차로 돌려 같은 결과가 나오는지 확인한다.
실물 / 모의를 반드시 나눠 적는다. 원본 로그 · 영상 경로와 커밋을 적지 못하면 그 줄은 `미확인` 으로 둔다.

## 양식

| 날짜 | 실행자 | 구분 (실물 · 모의) | 장비 | 설정 (명령 · 인자 · 계획 · 문제) | SW 버전 (ROS · 펌웨어 · 정책) | 커밋 | 실행 결과 (성공 · 실패 · 미확인) | 원본 로그 · 영상 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | |

## 기록

### 1. 주행 시연 drive_demo — 실물 (2026-11-15)

| 날짜 | 실행자 | 구분 | 장비 | 설정 | SW 버전 | 커밋 | 실행 결과 | 원본 로그 · 영상 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-11-15 (낮 ~ 밤) | WindForce08 (주행 담당) | 실물 | 실물 로봇 (Jetson · 벨트 v7 1 : 2.0) | `run_demo.sh real` 느린 · 구간마다 Enter · 계획 `7cm_실측L_어제순서` | ROS 2 lyrical (11-16 Jetson 로그 기준 · 11-15 판 미확인) · Nav2 RPP · 주행 OpenCR v5 (README 기준) | 미확인 (PR #306 머지 `e34af1c` 이전 로컬 판) | **실패** — S0 · S1 · S4 · S5 ✓, S2 · S3 정지점 5 ~ 8 cm, S6 벽 닿음 (경로에서 24 cm) | PR #306 본문 · `workspace/tools/drive_demo/README.md` 0 장. 원본 기록 (Jetson `~/demo_v2/runs/`) · 영상은 저장소에 없음 → 미확보 | 근거: https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306 . 실행자가 도구를 만든 사람이라 "다른 사람의 독립 재현" 으로 볼지는 팀이 정할 것. `test-results.csv` R-27 |

### 2. bringup 컨트롤러 활성화 — 실물 (2026-11-15)

bringup 은 kanichong 이 만들었고, 아래는 만든 사람이 아닌 팀원 (Onbeom) 이 같은 절차로 돌린 기록이다.

| 날짜 | 실행자 | 구분 | 장비 | 설정 | SW 버전 | 커밋 | 실행 결과 | 원본 로그 · 영상 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-11-15 16:03 ~ 19:40 | Onbeom (인지 · 위치 추정 담당) | 실물 | Jetson cart11 · 실물 로봇 (정지 상태) | `use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false` (11-14 실패 때와 같은 인자) | ROS 2 Lyrical | 설치본 c453996 (dev 1767e0f + STOW J1 −1.287) | **성공**: 3 판 모두 컨트롤러 4/4 · spawner 실패 0 · `/joy` 20 Hz · `/tf` 71 ~ 78 Hz (19:36 설치본 fd6bcbf 한 판 더, 같은 결과) | [#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6893200035) | 인지 · Nav2 끈 구성. 처음 두 번은 남은 노드 때문에 판정에서 뺌 |
| 2026-11-15 22:48 ~ 23:45 | Onbeom | 실물 | Jetson cart11 · 실물 로봇 (정지 상태) | `use_teleop:=false use_localization:=true use_nav2:=true use_planner:=false` + 인지 모드 7, 판마다 bringup 을 새로 띄움 | ROS 2 Lyrical | 설치본 1b9266c | **성공**: 8 판 모두 첫 bringup 에서 컨트롤러 4/4 (11-14 오전 같은 구성 30 번 중 25 번 실패) | [#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6896389227) | 원본 로그는 Jetson 에만 있음 |

### 3. 그 밖

| 항목 | 상태 |
|---|---|
| 전체 미션 실물 재현 (만든 사람 외 팀원) | **작성 필요** — 기록 없음. 11-16 경기 로그 (`real_20261116/`) 는 실행자가 로그에 남지 않아 재현 기록으로 쓸 수 없음 |
| 팔 주사위 배치 (`dice.sh`) 실물 재현 | **작성 필요** — 기록은 팔 담당 본인 실행뿐 (R-21 · R-22 · R-25) |
| 계획기 제출물 다시 만들기 (`mission_submit --problem config/problem_q1_example.yaml`) | **작성 필요** — 절차는 `project2/problem/README.md`, 다른 팀원이 돌린 기록 없음 |
| Isaac 평가 (`pallet_place_check.py --place` 등) | **작성 필요** — kanichong PR #313 · #320 결과는 만든 사람 본인 실행 |
