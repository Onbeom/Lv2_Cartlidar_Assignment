# mission 설정 정리 (#172 셋째 항목) — 2026-11-10

팀 저장소 작업 폴더 `~/git/Lv2_Cartlidar_Assignment_missioncfg`, 브랜치 `fix/mission-config-cleanup` (origin/dev e7cdde7).
빌드 · 시험 결과물은 이 폴더(`build/ install/ log/`)에 둔다. `run_checks.sh` · `bringup_static.py` 는 확인용이라 팀 저장소에 넣지 않는다.

## 바꾼 것

| 파일 | 내용 |
|---|---|
| `bringup/launch/bringup.launch.py` | `mission_source` 기본 `tag` → `plan`. `mission_table.yaml` 은 tag 일 때만 불러옴 |
| `mission/launch/mission_bringup.launch.py` | `source` 기본 `tag` → `plan`, tag 일 때만 `mission_table.yaml`, 잘못된 값이면 오류 |
| `mission/config/mission.yaml` | `align.coarse_distance` 0.45 그대로 두고 근거 주석, TODO 지움. 머리말 · `mission_source` 주석 |
| `mission/config/mission_table.yaml` | 머리말: 옛 태그 미션표, tag 모드에서만 불러옴 |
| `mission/src/mission_node.cpp` | 358 줄 TODO → 근거는 mission.yaml 에 있다는 주석 (동작 변화 없음) |
| `README.md`, `mission_planner/README.md` | 실행 · 운용 순서를 plan 미션 기준으로 |
| (안 바꿈) `mission.yaml` `diagnostics.timeout` 0.0 | 실물 `/diagnostics` 확인 (C 묶음 S1) 뒤 3.0 |

## coarse_distance 근거 (계산, 모의 · 실물 시험 아님)

- COARSE 거리 = 차체 앞 끝 → 앞 D435 가 추적한 퍽 (`align_controller.hpp` CoarseAlign: `ox - front_offset`).
- plan 모드 시작 박스: 퍽 2 개가 박스 중심 X ±40 mm, 면 → 퍽 0.1025 m. 조사에 쓰는 면(S·S·S·S·N·W)의 대기 자리 standoff 0.40 · 0.45 → 앞 끝 → 퍽 0.5025 · 0.5525.
- 하한: COARSE 동안 TFmini obstacle_stop(stop_m 0.25, 렌즈 = 앞 끝 − 0.0028)은 낮춰지지 않는다(FINE 만 낮춤). 끝난 자리 TFmini → 면 = d − 0.1025 + 0.0028 ≥ 0.25 + 여유 0.05 → d ≥ 0.40.
- 상한: d > 0.5025 − 허용오차 0.02 이면 대기 자리에서 후진 → d ≤ 0.48.
- READY `[0, 0.6, -1.2, -0.9, 0]` (URDF `waffle_4wd.urdf`, 충돌 상자 FK): 팔 끝 x 0.328 (앞 끝보다 0.195 m), 앞 끝 앞쪽 충돌 상자 최저 z 0.181 (base_link, 바닥 위 약 0.19) > 퍽 윗면 0.13 → 거리 제약 아님.
- 앞 D435 (피치 35.2°) 에서 d 0.45 의 퍽 중심은 광축 위 약 3°, 대기 자리는 약 7° → 수직 시야 안 (D435 컬러 약 ±21°).
- 결론: 0.45 는 [0.40, 0.48] 안이라 그대로 둔다.
- 남는 것: 퍽 2 개 중 하나를 따라 약 4° 돈다(yaw_tol 0.05 rad). 장소(빈 상자)는 퍽이 없어 COARSE 실패 → #160 (맡지 않음).

## 시험 (예상은 실행 전에 적음)

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| L1 | `mission_bringup.launch.py` 인자 없이 → `mission_source` | `plan` (기본이 plan) | plan — 통과 |
| L2 | L1 그대로 → `missions.tag_ids` · `locations.names` | 빈 목록 · `[home]` (미션표 안 불러옴) | `[]` · `['home']` — 통과 |
| L3 | L1 그대로 → `survey.enabled` · `verify.arm_pose` · `align.coarse_distance` | true · box_look · 0.45 (mission_plan.yaml 덧씌움) | True · box_look · 0.45 — 통과 |
| L4 | `source:=tag` → `mission_source` · `missions.tag_ids` | tag · [7, 12] (예전 tag 동작 유지) | tag · [7, 12] (위치표 7 개, survey 꺼짐) — 통과 |
| L5 | `source:=foo` | launch 오류 "source 는 plan 또는 tag" | `source 는 plan 또는 tag: foo`, 종료 코드 1 — 통과 |
| B1 | `bringup.launch.py` 기본 (구성만 계산) → mission_node 파라미터 · use_planner | mission.yaml + mission_plan.yaml + survey.pucks_out, mission_table 없음 · true | mission.yaml · mission_plan.yaml · {survey.pucks_out: pucks_file} · use_sim_time, use_planner true — 통과 |
| B2 | `bringup.launch.py mission_source:=tag` (구성만) | mission.yaml + mission_table.yaml · false | mission.yaml · mission_table.yaml · use_sim_time, use_planner false — 통과 |
| M1 | 미션표 없이 plan mock 미션 (mission.yaml + mission_plan.yaml, survey 끔, 주행 · 팔 · 인식 mock) | DONE (plan 모드가 미션표 없이 끝까지 감) | DONE, 계획 107 단계, 50.8 s (mock: navigate 18 · align 34 · skill 64 · check_dice 3) — 통과 |

- 모두 모의 확인(PC, 2026-11-10). 결과 파일 `checks/`. 실행: `./run_checks.sh` (도메인 78, 스스로 띄운 프로세스 그룹만 정리).
- B1 · B2 는 bringup 패키지를 빌드하지 못해(control · perception 등 설치본 없음) launch 파일을 불러 `control_container` 구성만 계산함(`bringup_static.py`). 실제 bringup 실행은 Jetson 에서.
- 안 한 것: Gazebo · 실물에서 COARSE 거리 확인(이 PR 은 값을 바꾸지 않음), `diagnostics.timeout` 3.0(실물 S1 뒤).
