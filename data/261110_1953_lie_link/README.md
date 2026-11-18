# #193 누운 직육면체 — 집기 순서 · 누운 퍽 스킬 (계획기)

- 작업 트리 `~/git/Lv2_Cartlidar_Assignment_lieorder`, 브랜치 `fix/lie-pick-order`(origin/dev 055f705). 고친 파일은 모두 kanichong 작성 계획기(사용자가 고치기로 결정 11-10, PR 전에 #193 에 알림).
- 사용자 결정(11-10):
  - 같은 상자에 서 있는 퍽과 누운 퍽이 있으면 서 있는 퍽 먼저.
  - 같은 자세끼리는 지금 순서(J1 거리 순).
  - 박스 6 inline(두 퍽 앞뒤)은 예외로 거리 순.
  - 순서는 계획기에서 바꿈.
- 고친 것:
  - `allocation.Puck.pose`(standing | lying, 등록표에 없으면 standing).
  - `planner._pick_tour` 정렬(inline 이 아니면 (서 있음 먼저, 거리)).
  - pick 단계에 `pose`.
  - `mission_lines.to_lines(lie_skill=)`: 누운 직육면체면 target_look 뒤에 그 스킬(세워 놓기 + 손 빼기 + 세운 자리 다시 재기) 한 줄을 넣고, 그 뒤 toppick_tilt · pallet_load 는 서 있는 퍽과 같음(11-11 사용자 결정: 다시 집기는 기존 고정 동작 toppick_tilt 재활용, n5 안 씀). 빈 값이면 넣지 않고 메모. 누운 원기둥은 메모만.
  - `planner_node` 파라미터 `lie_skill`(기본 빈 값).
- 그대로 둔 것(RL 쪽과 맞춤):
  - 직육면체 turn ±17.06°(누워도 돎, RL 이 시작 상태에 넣음).
  - target_look z 0.100(누운 퍽 스킬은 상자 기하 z 를 씀).
- 안 한 것: 등록표에 pose 쓰기(mission_node, PR #186 병합 뒤 따로), 스킬 이름(미정, #193).

## 시험 (계산 · 단위 시험, PC. 모의 미션 아님)

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| T0 | 기존 계획기 시험 `python3 -m pytest test` | 19 개 모두 통과(pose 없는 등록표는 standing 이라 결과 그대로) | 통과. 기존 19 개 + 새 7 개 = 26 passed |
| T1 | pose 읽기 | 없으면 standing, lying 읽음, 그 밖의 값은 MissionError | 통과(`test_parse_pose`) |
| T2 | 서 있는 퍽 먼저 | 모든 박스의 −X 퍽을 눕힌 경우와 +X 퍽을 눕힌 경우 둘 다, inline 아닌 정차에서 연달아 집는 두 퍽 중 누운 퍽이 먼저인 경우 0. 두 경우 중 한쪽은 원래 거리 순서를 뒤집어야 하므로 실제로 순서를 바꾼 정차가 1 개 이상 | 통과(단위). 단 `_pick_tour` 에 12 퍽을 한 번에 넣어 박스마다 두 퍽이 한 정차일 때만 본 것. **실제 계획에서는 거의 효과 없음** — 아래 '알게 된 것' |
| T3 | inline 예외 | inline 정차는 자세와 관계없이 J1 거리 가까운 퍽 먼저(앞 퍽이 누워도) | 통과(B6 inline 정차 2 번) |
| T4 | 칸 · 꺼내기 일관성 | 두 모드 모두 실은 칸 = 꺼낸 칸, 12 개, 꺼내기는 실은 반대(기존 규칙). 순서만 바뀌고 칸 배정 규칙은 그대로 | 통과(두 모드) |
| T5 | 스킬 문장 | lie_skill 주면 누운 직육면체만 target_look → 그 스킬 → toppick_tilt → pallet_load. toppick_tilt 는 12 줄 그대로, 누운 원기둥은 메모. 빈 값이면 그 줄만 빠지고 누운 직육면체마다 메모 | 통과(두 모드, 11-11 고친 뒤 26 passed). 처음 판(스킬이 toppick_tilt 를 대신)도 통과했었음 |
| T6 | 바꾸기 전과 같음 | pose 없는 예제(problem_q1_example)로 origin/dev 계획기와 이 브랜치 문장 목록이 두 모드 모두 한 줄도 다르지 않음 | 통과. per_location 105 줄 · single_load 99 줄 모두 같음(origin/dev 를 git archive 로 꺼내 비교) |

## 알게 된 것 (11-10)
- 계획기는 같은 상자의 두 퍽을 대부분 **다른 정차**에서 집는다.
  - per_location: 두 퍽이 다른 장소로 가면 다른 운반 회차에서 집는다.
  - single_load: 두 퍽이 다른 팔레트 열이면 다른 싣기 단계에서 집는다.
  - 예제(problem_q1_example)에서 한 정차에 두 퍽이 같이 있는 상자는 per_location 0/6, single_load 1/6(B2) 뿐이다.
- 그래서 정차 안 정렬만으로는 규칙이 거의 지켜지지 않는다. 예제에서 퍽 하나만 눕힌 12 가지마다 "누운 퍽을 같은 상자 서 있는 퍽보다 먼저 집음"을 세면 per_location 6/12, single_load 5/12 다(이 브랜치 계산).
- 원래 있던 일: single_load 예제에서 B6 inline 의 먼 퍽 11(2 열)을 가까운 퍽 10(1 열)보다 먼저 집는다. 정차 안 '가까운 퍽부터'는 열 단계를 넘어서는 지키지 않는다.
- 규칙을 계획 전체에 지키려면 계획을 고르는 단계(장소 순서 permutations · 열 나누기 _row_splits)에 제약이 필요하다. 제약은 "누운 퍽은 같은 상자 서 있는 퍽보다 뒤, inline 은 가까운 퍽이 먼저"이고, 만족하는 계획이 없으면 비용 최소 계획 + 경고로 간다. 사용자 결정 대기.

## 11-11 바꾼 것
- RL 모의: dev 는 `skill.enabled: false` 라 미션 toppick_tilt = 고정 동작 집기. 세운 퍽을 이것으로 다시 집기 60/64(기본 배치 61/64), `261110_2010_lie_rl/README.md` 5-2절.
- 그래서 누운 퍽 스킬은 세워 놓기까지만 하고 다시 집기는 기존 toppick_tilt 로(사용자 결정). 세운 자리 D 는 target_look match_distance 0.04 m 밖일 수 있어 스킬 끝에 D 를 target_look 으로 다시 잼(toppick_tilt 는 직전 target_look 값 사용).
- 다시 확인: pose 없는 예제 origin/dev(7cb49b0) 와 문장 같음(105 · 99 줄). 등록표 연결 U4(`261110_2242_survey_pose`) 도 이름을 `lie_upright` 로 다시 돌려 두 모드 1 줄씩.
