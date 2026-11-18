mock 미션 확인했습니다 (PC 모의, dev cf358ff, planner + mission_node + `test/mock_robot.py`, survey 끔).

| 판 | 조건 | 결과 |
|---|---|---|
| R0 | start_box · oblique 끔 | DONE |
| R1 | 둘 다 켬, 시작 박스 2 cm · −1.5 cm · 12° | DONE. 시작 로그의 geom (j1_x 0.0960 · center_x −0.04035 · box_x 0.3625, box_half 0.1025 · tfmini_x 0.1300) · robot_geometry (front_offset 0.1328 · icr_x −0.0658) 값이 파일과 같음. measure_front 13 번, 바뀐 target_look:odom 12 개가 mock 참 퍽 자리와 최대 0.05 mm |
| R2b | 둘 다 켬, 돌림 42° | DONE, '±45° 부호 모호' 경고 (멈추지 않음) |
| R3 | robot_geometry 복사본 front_offset +10 mm | 시작 로그 0.1428, 보기 자리 align BOX 0.353 → 0.363 m (+10 mm) → front_offset 을 파일에서 읽는 것 확인. geom 값(j1_x · center_x · box_x · box_half · tfmini_x)은 파일 값과 같은 것만 봤고, 바꿔 보지는 않음 |

- 돌림을 정확히 40° 로 주면 경고가 안 납니다 (각을 rad → deg 로 바꾼 값이 40 바로 아래). 경계라 동작에는 문제 없다고 봅니다.
- 한계: mock_robot 의 measure_front 는 장소 상자를 흉내 내지 않고 가장 가까운 시작 박스를 내서, 장소 L1 ~ L3 의 비스듬히 서기 재기는 max_shift 로 실패 → 지도 좌표로 놓는 대체 흐름만 봤습니다. 회전 · 팔 재기 흐름과 실물은 아직입니다.
