# 로봇 없이 낼 수 있는 Issue · 일 목록 (11-13, 실물실험 261112 3)
| # | 무엇 | 담당 | 초안 | 상태 |
|---|---|---|---|---|
| I1 | 직진 중 오른쪽 옆 미끄럼 2.35° (1 m 에 4.1 cm, 실물 측정, 바닥 경사 미확인 — 180° 돌려 가르는 시험 부탁) | 바퀴 WindForce08 | `issue_drive_straight_drift.md` | **올림 #248** |
| I2 | 실물 xacro 와 Isaac 모델의 라이다 자리 24.3 mm 차이(브라켓 v2) — test_model_compare dev 실패 원인 | Isaac 모델 kanichong · xacro 사용자 | `issue_laser_bracket_v2.md` | **올림 #249** |
| P1 | 팔 place_box 배율 5 → 3 개 (Jetson 장당 4.77 s → 약 2.9 s 계산) | perception (우리) | PR 본문 `../pr/pr_body_scales3.md`, 브랜치 fix/place-box-arm-scales3 | **PR #250**(e575a22) |
| D1 | 시작 박스 '면에 수직 다시 서기' 제안 (#226 다음 일) | mission (우리, #247 위) | `../../261112_2015_start_box_yaw/proposal_restand.md` | 조건부 승인 → 모의 R1 ~ R3 진행 |
- 실물이 필요해 새 세션으로 넘긴 것: 뒤 D435 피치 RD2 · R2 원인(앞바퀴 x · 상자) · #226 K1 ~ K3 · 6 단계 팔 보정 결과 풀기.
