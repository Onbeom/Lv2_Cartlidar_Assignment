# Nav2 · AMCL 인계 묶음 (2026-11-03, 인지 파트 → 주행 · Nav2 담당)

이 묶음을 받은 Claude 에게: 이 파일을 먼저 읽고, 아래 순서대로 읽으면 된다. 수치는 모두 **Isaac Sim 모의**(B안 URDF, 발제 미로 seed 1)에서 잰 것이고 실물 값이 아니다.

## 무엇을 넘기나
| 넘기는 것 | 지금 상태 | 받는 쪽이 할 일 |
|---|---|---|
| **Nav2 설정 · 보정 노드**(설계 45) | 모의 판정 통과(아래). 팀 저장소에는 아직 Nav2 launch · params 가 없음 | 팀 저장소에 Nav2 launch · params 로 넣고 실물에서 조정 |
| **AMCL 노드 운용**(설정 · launch · 실물 조정) | 최적화 설정(`amcl/amcl.yaml`) 모의 통과 | Nav2 launch 에서 AMCL 을 띄우고 실물에서 조정 |

넘기지 않는 것(인지 파트가 계속 맡음): 팀 저장소 `localization` 패키지 — `loc_monitor`(위치 잃음 감지 · 다시 찾기, `/localization/status` · `/localization/lock`), 지도(`/map` · `/map_objects`), AMCL 입력 스캔(`/scan_merged`). 경계는 `01_interface.md`.

## 읽는 순서
1. `01_interface.md` — 누가 어떤 토픽 · TF 를 내는지, 받는 쪽이 지켜야 할 규칙(loc_gate)
2. `02_amcl.md` — AMCL 설정: kanichong 원본(`mission_planner/config/amcl.yaml`)에서 바꾼 5 개와 근거, 결과
3. `03_nav2.md` — Nav2 구성 · 보정 노드마다 이유 · 판정 기준 · 결과
4. `04_known_issues.md` — 알려진 문제 · 실물에서 다시 정할 값 · 다른 담당과 엮인 일

## 폴더
```
amcl/amcl.yaml                          AMCL 최적화 설정(frame map, scan /scan_merged, map /map)
nav2/config/nav2_params.yaml            Nav2 설정(설계 45, B안 footprint, frame map, amcl 절 없음) — @NAV2_CONFIG_DIR@ 두 곳 바꿀 것
nav2/config/bt/…replan_only_after_failure.xml   BT
nav2/config/lattice_straight_rot_arc25_0025.json Smac Lattice 동작 표(직선 · 제자리 회전 · 반경 0.25 호, 0.025 m)
nav2/nodes/turn_floor.py                속도 명령 보정 노드(제자리 회전 바닥값 · 앞먹임 · frame 이름) — Nav2 체인 안
nav2/nodes/icr_odom.py                  참고: 회전 중심 보정 odom(bringup EKF 쪽 일, 04 참고)
nav2/launch_reference/nav2_reference.launch.py  모의에서 쓴 launch(경로가 작성자 PC 절대 경로, 구성 참고용)
reference/design_docs/                  설계 기록(값마다 이유 v21 → v45, B안 · AMCL 최적화 기록)
reference/sim_tools/                    모의 채점 도구(작성자 PC 의 Isaac 장면 전용, 판정 기준 정의 · loc_gate 구현 참고)
```

## 모의 결과 한눈에(판정 기준 G1~G5 는 `03_nav2.md`)
- Nav2 설계 45 + Cartographer: 3 판 G1~G5 통과(위치 오차 RMSE 0.0072~0.0081 m).
- Nav2 설계 45 + AMCL 최적화 설정: 3 판 G1~G5 통과(RMSE 0.0154~0.0162 m, 최대 ≤ 0.030 m). kanichong 원본 설정은 1 판 RMSE 0.0317 · 최대 0.095 m, G3(차체 여유) 불통과.
- 팀 저장소 `localization` 패키지 + 팀 bringup EKF + AMCL: K0(정상 주행) G1~G5 통과지만 SUSPECT 2 회, K1(정지 중 0.20 m · 10° 강제 이동) 감지 0.52 s · 다시 찾은 오차 0.0156 m 통과.
