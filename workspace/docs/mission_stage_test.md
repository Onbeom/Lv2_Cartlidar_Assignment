# 미션 실물 단계별 시험 (Jetson, #184 S7 준비)

2026-11-14 작성. 미션을 처음부터 끝까지 한 번에 돌리기 전에 앞 단계부터 하나씩 클리어한다.
각 단계에서 막히면 그 단계 이슈로 돌아가고, 결과는 #184 에 남긴다 (표 맨 아래 형식).

## 0. 준비 (한 번)

| 할 일 | 방법 |
|---|---|
| 빌드 = 실물 벨트 | v7 (1:2.0) 실물 장착 (2026-11-15) → 코드 `wheel_gear` 2.0 그대로 빌드. Nav2 한 프로세스 (`nav2_composed`, #276) 때문에 `navigation` 도 빌드 |
| 퍽 등록표 | 측량 자세 (#154) 를 정하기 전에는 측량을 끄고 등록표로 계획한다. **`workspace/docs/stage_test/pucks_stage.yaml` 대로 경기장에 퍽을 놓는다** (아래 배치표). 바꿔 놓으면 파일도 같이 고침. 신호 · 바코드는 관측값을 쓰므로 파일 값은 무시됨 |
| 팔 대기 자세 | 끄기 전 STOW 로 접어 둠. 손으로 놓을 때 범위 J1 99 ~ 112.2° · J5 −17.5 ~ +17.4° (#305, 4b76631). 밖이면 opencr_hardware 가 시작 거부 → 다시 놓기 |

배치표 (`pucks_stage.yaml`, 모두 세움, 왼 = 박스 중심 X − 40 mm · 오른 = X + 40 mm, 지도 X 방향 기준):

| 박스 | 중심 (m) | X − 40 mm (side −1) | X + 40 mm (side +1) |
|---|---|---|---|
| 1 | (1.468, 0.950) | 빨강 원기둥 | 초록 직육면체 |
| 2 | (3.670, 3.152) | 빨강 직육면체 | 초록 원기둥 |
| 3 | (2.936, 5.354) | 빨강 원기둥 | 초록 원기둥 |
| 4 | (0.734, 5.354) | 빨강 직육면체 | 초록 직육면체 |
| 5 | (2.202, 5.354) | 빨강 원기둥 | 초록 원기둥 |
| 6 | (3.670, 3.886) | 빨강 직육면체 | 초록 직육면체 |
| 기록 위치 | `log_dir` 기본 `~/Lv2_Cartlidar_Assignment/mission_log/<YYMMDD_HHMM>/` (mission_log.jsonl · 사진 · 계획기 기록) |

공통 실행 (아래 `<단계>` 만 바꿈):

```bash
source workspace/ros2_ws/install/setup.bash
ros2 launch bringup bringup.launch.py use_survey:=false pucks_file:=$PWD/workspace/docs/stage_test/pucks_stage.yaml planner_stop_after:=<단계>
```

시작 전 매번 (README "운용 순서"):
1. 팔 STOW, 로봇을 출발 자세 (0.694, 0.950, yaw π) 에 놓기
2. 터미널 `[준비 확인]` 표가 `READY n/n` 이 될 때까지 기다림 (E-stop 은 조이스틱 리셋 전까지 `[실패]`). 다른 터미널: `ros2 topic echo /mission/ready --once`
3. 조이스틱 리셋 → 팔 STOW → DRIVE 펴짐 확인 → 시작 버튼
4. 진행: `ros2 topic echo /mission/status`

## 1. bringup 반복 (#275 · #276)

```bash
workspace/ros2_ws/src/bringup/test/bringup_repeat.sh 10 -- use_teleop:=false use_survey:=false pucks_file:=$PWD/workspace/docs/stage_test/pucks_stage.yaml
```
- 통과: 10 / 10 컨트롤러 4 개 active · /joint_states. 같은 판 로그에서 `[준비 확인]` 마지막 표의 2 컨트롤러 · 5 Nav2 줄도 셈 (`grep -A20 "준비 확인" bringup_N.log | tail -20`)
- 실패 판: 표의 `[실패]` · `[대기]` 항목이 어느 단계인지 (#275 컨트롤러 · #276 Nav2 · #265 부하). Nav2 실패면 `grep -h "not available after waiting\|Aborting bringup\|client will not receive response" ~/.ros/log/latest/*.log` 로 Nav2 (`nav2_composed`) 쪽인지 위치 추정 (`lifecycle_manager_maps`) 쪽인지 #276 에

## 2. observe (신호 · 바코드)

`planner_stop_after:=observe` — 주행 없음. 팔 TAG_VIEW → 신호 12 점멸 · 바코드 읽기 → 계획 → wait → DONE.
- 확인: 로그 signal · barcode 값이 실제와 같음, 계획기 `planner/allocation.json` 이 손 계산과 같음
- 막히면: 인지 (신호 MISMATCH 4 번 = FAILED), AMCL 초기화 (map TF 없음)

## 3. first_pick (박스 1 곳 집기)

`planner_stop_after:=first_pick` — 박스로 주행 → COARSE → ALIGN → FINE → (turn) → target_look → toppick_tilt → pallet_load → turn_0 → HOME 복귀.
- 확인 (손으로 기록): FINE 뒤 TFmini 거리 (목표 0.13 m, #87), 집기 성공 · 칸에 들어감, 팔이 차체 · 팔레트에 닿지 않음 (#219 · #277), HOME 에서 return_check 판정
- 실패하면: 재시도 2 번 뒤 집기 실패는 그 퍽 건너뜀 (`skipped: pick r_i`), 주행 · 정렬 · pallet_load 실패는 HOME 복귀 뒤 FAILED — reason 을 그대로 기록

## 4. first_place (장소 1 곳 놓기 + verify)

`planner_stop_after:=first_place` — 첫 장소에 놓을 퍽 모두 집기 → 장소 이동 → 상자 재기 → pallet_unload → place → verify → HOME.
- 확인: 놓은 칸 · 모양이 기대와 같음 (사진 images/), verify 결과 (check_dice), 상자 재기 실패 시 지도 좌표 사용 여부 (로그 measure_box)

## 5. 전체 (none)

`planner_stop_after:=none` (기본). 시간 · 건너뛴 퍽 · 복귀 판정까지 기록 → #184.

## #184 에 남길 형식

| 날짜 | 단계 | 결과 (DONE · FAILED · skipped) | 걸린 시간 | 막힌 단계 · reason | 기록 폴더 | 메모 |
|---|---|---|---|---|---|---|

## 같은 절차의 PC 확인 (2026-11-14, mock)

계획기 + mission_node + `mission_planner/test/mock_robot.py` (Q1 예제, 측량 끔):
observe → DONE (skill 1) · first_pick → DONE (주행 2 · 정렬 2 · skill 7) · first_place → DONE (주행 7 · 정렬 10 · skill 25 · check_dice 1) · none → DONE.
실패 주입: 한 퍽 target_look 거절 → `skipped: pick 1_1` DONE, 놓기 거절 → HOME 복귀 뒤 FAILED.

2026-11-15 다시 (dev e34af1c + 4b76631, 등록표 `stage_test/pucks_stage.yaml`): observe DONE (skill 1) · first_pick DONE (주행 2 · 정렬 2 · skill 7) ·
first_place DONE (주행 7 · 정렬 10 · skill 25 · check_dice 1 · place_box 1) · none DONE (주행 16 · 정렬 25 · skill 64 · check_dice 3 · place_box 3).
