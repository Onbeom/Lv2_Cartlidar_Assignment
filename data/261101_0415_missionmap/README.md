# 측량 좌표계 mission_map · 2026-11-01 (YOLO 1101 2 통보 → 사용자 지시대로 묻지 않고 바로 고침)

통보(1단계 SLAM 고정): TF 트리 mission_map →(정적) map →(Cartographer · relay) odom →(EKF) base_link. mission_map = 발제 좌표, map = Cartographer 지도(원점이 지도마다 다름). 현재 지도 align.json = x 0.729 · y 0.960 · yaw 3.143774.
문제: 측량(`puck_survey.yaml`) `world_frame: map` → 칸 좌표(발제)와 지도 좌표가 달라 관측이 모두 칸 밖(outside_slot)이 됨.

고침(브랜치 `fix/perception-survey-mission-map`, #24 head e23f848 위 — puck_survey.yaml 이 #24 에서 생김, 작업 트리 `~/git/Lv2_Cartlidar_Assignment_missionmap`, 패치 `missionmap.patch`): `world_frame: mission_map` + perception_survey README 한 줄.
도구 수정: `replay_b0.py --mission-map X Y YAW`(mission_map → map = A, map → odom = A⁻¹, 역변환 오차 1.1e-16), `run_replay.sh REPLAY_SURVEY_SET`(측량 설정 사본 값). 고치기 전 사본 `before/`.
빌드: `build_branch`(e23f848 + 변경), `build_merged`(756c2b4 + #32 · #33 · TFmini 끊김 · 이 변경, 작업 트리 `_merge13`).

| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| 단위 시험 | 모두 통과 | 브랜치 · 합친 트리 모두 이전과 같은 개수(설정만 바뀜) | **브랜치 113/113**(#24 기준, perception_survey 시험 포함), **합친 트리 162/162** |
| 대조(도구 확인): 옛 설정 map + SLAM 꼴 TF, seed 101 T6500 2회 | 실패해야 함 | 0/2, 관측 모두 outside_slot — 결함 재현 | **0/2 · 0/24칸**, 등록된 관측 0 · 칸 밖 65 · 68 — 재현 |
| F1c: 새 설정 mission_map + SLAM 꼴 TF, B0′ 24회 | F1 과 같음(24/24 · 288/288 · 틀린 완료 0 · 틀림/모름 0 · ≤ 10 mm · 무응답 0) | 24/24, 칸 · 오차 F1b 와 같음 — mission_map → odom 이 항등이라 측량 입력이 같음 | **24/24 · 288/288**, 틀린 완료 0, 틀림 0, 최대 5.35 mm, 무응답 0. 완료 파일 frame `mission_map(mm, 발제 원점)` |

결과 `neg/` · `f1c/replay_*/summary.json`(2026-11-01). **F1c 통과(모의).** 열린 PR 11개 head 와 병합 충돌 0. PR 본문 `PR_missionmap.md`.
