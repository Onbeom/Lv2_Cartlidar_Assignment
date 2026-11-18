# 시작 박스 '면에 수직 다시 서기' (start_box.restand, #247 다음, 11-13 01시~, 모의)

- 제안 · 조건부 승인: `../261112_2015_start_box_yaw/proposal_restand.md` (#247 설계 · 결과는 같은 폴더 README).
- 바탕: worktree `~/git/Lv2_Cartlidar_Assignment_restand`, 브랜치 `fix/mission-start-box-restand` (← `fix/mission-start-box-yaw` 2a0f248 = #247, #229 · dev 합침 포함). 커밋 안 함.
- 이 폴더 도구: `r1_calc.py` (R1 계산), `run_mock.sh` · `run_r2_all.sh` (R2 · R3 모의 미션), `check_r2.py` (R2 판정), `check_r3.py` (R3 비교).

## 설계
1. 언제: `start_box.enabled` + `start_box.restand` (기본 false) 일 때 measure_start 성공 뒤. 그 접근 (다음 goto 전) 의 직육면체 집기
   (= 앞에 `turn_<p|m>` 줄이 있는 target_look) 마다 '계획 final 에 섰을 때 #247 회전으로 남을 면 어긋남 r' 을 미리 셈 (#247 `sb_choose_turn` 그대로).
   r 하나라도 `restand_deg` (20°) 를 넘으면 다시 서기 후보.
2. 새 자리: 자세표 접근 (final · stage) 을 상자 지도 중심 (사전값) 둘레로 잰 돌림 rot 만큼 돌리고 잰 중심으로 옮김.
   rot 은 #247 과 같이 지도 축 기준 ±45° 로 접은 값이라, 원래 접근 방향에 가장 가까운 면의 바깥 법선 위 · 같은 gap (0.1272) · 같은 standoff.
   퍽은 상자 틀에서 그대로 (±0.04) 이므로 새 자리에서는 계획기와 같은 turn (±17.06°) 이 면에 맞음 (β' = 0).
3. 순서: `measure_start` 뒤에 `restand <B<n>:<면>>` 단계를 끼움 = 새 stage 로 /drive/navigate + 팔 주행 자세 → 원래 `approach` (FINE) → 원래 집기 줄.
   turn 은 #247 이 FINE 자리에서 다시 고름 (범위는 막힘 판정에서 쓸어 본 ±22.06° 로 좁힘), target_look 은 #247 대로 odom 자리. 다음 goto 는 계획 그대로.
   `restand` 는 goto 가 아니라서 #247 의 '접근 묶음' (findStartMeas) 이 끊기지 않음. 새 stage 주행이 실패하면 한 번은 계획 stage 로 가서 #247 회전 (기록).
4. 조건 1 (부호 모호): |rot| ≥ `sign_warn_deg` (40°) 면 다시 서지 않음 → #247 회전 + 경고 · 기록 (`decision: ambiguous`). 정사각 윗면으로는
   +45° 와 −45° 를 못 가려 면 · 퍽 자리가 90° (57 mm) 틀릴 수 있는데, 틀린 면에 다시 서면 회전만 할 때보다 나빠질 수 있어서.
5. 조건 2 (막힘): 계획기 지도 · 외곽으로 ① 새 final 외곽 + 0.05 ② stage → final 곧은 길 (외곽이 쓸고 가는 사각형) + 0.05 ③ 새 final 에서 집기 회전
   (로봇 중심 둘레 ±22.06°, 10 등분) 외곽 + 0.01 (계획기 turn_range 여유) 이 벽 · 기둥 · 다른 박스와 안 겹쳐야 함. 대상 박스는 뺌 (잰 상자 면에서 gap 0.1272 로 섬).
   막히면 #247 회전 + 기록 (`decision: blocked`, `block: final | path | turn`).
   - **방법 선택**: mission (C++) 은 계획기 지도를 못 읽음. 그래서 계획기 기하 (arena.load_sdf · World 와 같은 높이 거름 · poses 외곽 · 분리축) 로
     박스마다 근처 장애물 다각형 (상자 중심 반경 1.1 m, 대상 박스 뺌, 겹친 기둥 받침 · 몸 · 머리 하나로) 을 `mission/config/restand_obstacles.yaml` 에 미리 적고,
     mission 은 잰 자리 그대로 같은 식 (`restand.hpp rs_blocked`) 으로 판정함. '보수적 범위 (restand_ok_deg)' 보다 정확하고 (잰 밀림 · 돌림 그대로),
     계획기 코드 (kanichong) 를 고치지 않음. 자세표가 바뀌면 자료의 final · stage 가 달라지므로 mission 은 그 접근을 다시 서지 않고 경고.
   - 생성기: `mission/tools/make_restand_obstacles.py` (계획기 모듈을 import 만 함). R1 은 이 다각형 판정과 계획기 Solid 직접 판정 (footprint_clear) 이 같은지도 봄.
6. 조건 3 (시간): 다시 서는 접근마다 navigate + approach 한 번 더 (약 20 ~ 40 s). 미션 전체 최악 = (다시 설 수 있는 접근 수) × 40 s (아래).

## 판정 기준 (승인) · 결과

| # | 확인 | 기준 | 예상 | 결과 |
|---|---|---|---|---|
| R1 계산 | 8 개 ok 시작 박스 접근 × β ±25 ~ ±45 (5°) × 밀림 (dx, dy) ∈ {−0.05, 0, 0.05}² = 720 경우 (잰 = 참) | 막히지 않고 모호하지 않은 경우 다시 선 뒤 남는 어긋남 ≤ 5°, 막힘 (final · 길 · 회전) · 모호 (\|β\| ≥ 40) 수와 돌아간 경우 #247 어긋남 기록 | (제안 때) 대부분 막히지 않음 (근처에서 닿을 수 있는 것은 대상 박스뿐), 좁은 B3 · B6 는 일부 막힐 것 || **통과** (기준) · 예상과 다름 (막힘이 많음). 720 경우 중 S/N 630: 20° 넘음 476, 그중 모호 252, 막힘 160 (final 144 · 길 16 · 회전 0) → 다시 섬 64. 다시 선 뒤 최대 r 0.001° (≤ 5°, 실패 0). 다시 서지 못한 경우 #247 r 최대 34.0°. 다각형 자료 판정 = 계획기 Solid 판정 720/720. β 0 은 8 곳 모두 안 막힘. 다시 설 수 있는 것은 열린 쪽만: B1:S · B2:S · B4:N · B5:N 은 β +25 ~ +35, B4:S 는 −25 ~ −35. B3:S · B3:N 은 양쪽 모두 막힘 (±25° 부터 옆 벽판에 외곽이 겹침, 여유 0). B6:W (inline) 는 turn 줄이 없어 대상 아님. 표 `r1_table.md`, 경우별 `r1_result.json` |
| R2 모의 미션 | #247 mock (Q1 예제) 에 β +30 · −30 · +45 · −45 · −35, 밀림 (+0.05, +0.05) β +30, 덧붙여 다시 서기 주행 실패 (β +30) | DONE, 다시 선 집기의 남는 어긋남 ≤ 5° (mock 참 상자 · 실제 돈 자세로 따로 셈), \|β\| ≥ 40 → 다시 서지 않음 + 경고, 막힘 → #247 로 돌아감 + 기록 | 통과. β +30: B1:S 다시 섬 (r 23.7 → 0), B4:S 오른쪽 막힘 (final) → 회전 r 23.7. β −30: B4:S 왼쪽 다시 섬, B2:S 막힘 (final). ±45: 다시 서기 0, 경고 4 줄. −35: B4:S 길 막힘 (path). 밀림 +30: B1:S · B2:S 다시 섬 (B2:S r 20.4). 주행 실패 (β +30, mock 이 자세표 밖 navigate 거절): B1:S 계획 stage 로 돌아가 회전 r 23.7 + restand_fallback 기록, DONE || **통과** (7 경우 모두 DONE, check_r2 실패 0 — navfail 의 '기대 restand 인데 회전만' 1 줄은 의도한 되돌림). b30: B1:S 다시 섬 (turn_m17.1, r 0.019°), B4:S 오른쪽 blocked final → turn_m30.0 r 23.66. bm30: B4:S 왼쪽 다시 섬 (r 0.02°), B2:S blocked final. b45 · bm45: 다시 서기 0, ambiguous 4 묶음 (경고 줄 '다시 서지 않음' 4 + 부호 모호 10). bm35: B2:S final · B4:S path 막힘 → 회전 r 28.37. s30 (+0.05, +0.05): B1:S (r 32.68 → 0.019) · B2:S (20.38 → 0.02) 다시 섬, B4:S 둘 blocked final. navfail: restand_fallback 기록 → 계획 stage → turn_m30.0 r 23.66, DONE (navigate 18 = 16 + 실패 1 + 다시 1). 모든 경우 target_look 참 퍽에서 ≤ 0.068 mm, mission 결정 = 따로 셈한 기대 (계획기 Solid · 다각형 둘 다 같은 종류), 다시 선 FINE 자리 = 계산 자리 2 mm 안 |
| R3 꺼짐 회귀 | `restand` false (enabled true, β +30 · 같은 mock) · enabled false | 보낸 skill · navigate · align · pose · set_mode · check_dice 열이 #247 빌드 (startbox worktree, 같은 mock) 와 같음 | 같음 || **같음**. restand false (enabled true, β +30) 와 #247 빌드: navigate 16 · align 25 · drive_skill 8 · pose 44 · skill 64 · set_mode 106 · check_dice 3 · place_box 3 모두 같음, DONE. enabled false 도 같음 (set_mode 96) |
| 기존 시험 | colcon test mission (gtest, test_restand 6 개 더함) · mission_planner (pytest) | 모두 통과 | 통과 (test_restand: 자세 · 미리 셈 (#247 S2 값 23.66 · 8.54) · 분리축 · 막힘 종류 · 결정 · R1 표본 18 경우가 다각형 자료로 같은 종류) || **통과**. colcon test 104 개 0 실패 (mission gtest 7 묶음 40 개: test_restand 6 포함, mission_planner pytest 57, CTest 묶음 7) |
| 시간 | Q1 예제 · 일반 미션 | 다시 서는 접근 수 × 20 ~ 40 s 기록 | Q1 은 회전 집기 묶음 4 개 → 최악 4 × 40 = 160 s || Q1 은 회전 집기 묶음 4 개 (B1:S · B2:S · B4:S 두 번). 모의에서 다시 선 묶음 0 ~ 2 개 (β +30: 1, −30: 1, 밀림 +30: 2, ±45 · −35: 0). 최악 (4 묶음 모두) 4 × 20 ~ 40 = 80 ~ 160 s. R1 로는 B3 · B6:W 는 다시 서지 않으므로 Q1 배치에서 실제 최악도 4 묶음. 일반 미션은 'S/N 접근 중 직육면체 집기가 있는 방문 수' × 40 s (예: 직육면체 6 개가 모두 따로 방문이면 240 s). 새 stage 는 원래 stage 에서 0.32 ~ 0.45 m (R1 64 경우) |

### 정리 · 한계
- 다시 서기가 되는 범위가 좁음: 시작 박스가 미로 통로에 있어 β 가 '벽 쪽' 으로 돌면 새 final 외곽이 옆 벽판에 겹침 (여유 0). 그래서 '불리한 쪽' 이 열린 쪽과 같을 때만 다시 섬 (R1 S/N 의 20° 넘는 224 경우 (모호 제외) 중 64).
  나머지는 #247 회전으로 r 20 ~ 34° (k9 모의 20 ~ 30° 77.9 %, 30 ~ 45° 63.2 %).
- 모의 mock 의 FINE 은 '요 · 옆 자리는 그대로, 앞으로 참 면까지' 로 둠. 실물 drive FINE (Arducam 좌우 + TFmini) 이 비스듬한 상자 앞에서 어떻게 서는지는 확인 안 함. 잰 상자 = 참 상자로 둠 (잰 오차는 R1 · R2 에 없음).
- inline 접근 (B6:W) 의 직육면체는 계획 줄로 원기둥과 구별이 안 돼 대상 아님 (#247 과 같은 한계).
- 막힘 자료는 자세표 · SDF 가 바뀌면 `python3 workspace/ros2_ws/src/mission/tools/make_restand_obstacles.py` 로 다시 만들어야 함 (안 맞으면 mission 이 그 접근을 다시 서지 않고 경고).
- 회전 막힘 (turn) 은 R1 에서 0 경우 → gtest 의 합성 장애물로만 확인.
- 다시 서기 주행 실패 되돌림은 mock 의 navigate 거절로만 확인 (Nav2 실제 실패 아님).

## 처리 (검토 261111 1 승인, 11-13)
- **다시 서기는 보류**: PR 올리지 않음. 브랜치 `fix/mission-start-box-restand` 는 로컬에 커밋 전으로 둠(남이 쓴 줄 3 개도 올리지 않음). 이유: 20° 넘는 476 경우 중 다시 섬 64(13 %) — 막힘 160 · 부호 모호 252. 얻는 것에 비해 지도 자료 유지 · 남의 줄 변경이 큼.
- 차선 손목 면 맞춤은 Issue 로 팔 담당에게(`issue_wrist_face_align.md`).
