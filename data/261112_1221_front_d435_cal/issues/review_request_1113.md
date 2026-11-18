[실물실험 261112 3 → 검토 261111 1] 실물 없는 4 건 검토 요청 (11-13 01시 반, 검토 세션이 꺼져 있어 파일로 둠)
- 목록: `261112_1221_front_d435_cal/issues/ISSUE_LIST.md`. 승인되면 바로 올림.
- I1 바퀴 Issue(WindForce08) `issue_drive_straight_drift.md`: 직진 앞 · 뒤 모두 오른쪽 옆 미끄럼 2.25 ~ 2.42°(1 m 4.1 cm, 사람 4.5 cm), 구간 중 돈 양 −0.97 ~ +0.75°. 조건(카펫 · 0.05 m/s · 경사 미측정) · 짐작은 '확인 안 함'.
- I2 라이다 자리 Issue(kanichong) `issue_laser_bracket_v2.md`: 실물 xacro laser x ±0.1089 · z 0.0755 vs Isaac(a1cea6e 브라켓 v2) x ±0.1318 · z 0.0675 → 24.3 mm, test_model_compare dev 실패 원인. 실물에 v2 인지 물음.
- P1 팔 place_box 배율 PR(`fix/place-box-arm-scales3`, 커밋 전): [0.94 … 1.06] → [0.96, 1.0, 1.04]. 모의 ±0.01 112 장 15 mm 안 · 가림 틀린 값 0 · 회귀 같음 · 단위 229/229, 기록 장(−0.02 m) 1 장 26.6 mm 틀림(한계에 적음). frames 3 → 2 는 안 함(기준이 장당 시간). 본문 `../pr/pr_body_scales3.md`.
- D1 '면에 수직 다시 서기' 제안 `../../261112_2015_start_box_yaw/proposal_restand.md`: 남는 각 > 20° 면 잰 상자의 가까운 면 법선 위 같은 standoff · gap 으로 다시 접근(막히면 #247 회전). 기준 R1 계산(남는 각 ≤ 5°, 막힘 수 기록) · R2 mock · R3 꺼짐 회귀.
