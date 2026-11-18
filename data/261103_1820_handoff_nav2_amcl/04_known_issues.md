# 04. 알려진 문제 · 실물에서 다시 정할 값 · 다른 담당과 엮인 일

## Nav2 약점(모의에서 봄, 고치지 않은 채 넘김)
1. **출발 칸 → 첫 목표 구간 흔들림**: 14 판 모두 출발 칸 모퉁이에서 앞뒤로 흔들며 나감(출발 → x > 1.0 m 까지 27.7~41.6 s, 최소 여유 0.009~0.044 m).
   1 판(팀 localization 패키지 + ICR EKF + Cartographer)은 진행 검사(0.25 m · 15 s)에 걸려 중단 → 그 자리(실제 여유 0.051 m, 겹침 없음)가 costmap 상 점유라 'Start occupied' 로 뒤 목표 6 개 계획 실패, 복구 동작 없음.
   위치 추정 오차는 그 판에서 작았음(RMSE 0.0083 m) → Nav2 쪽 문제. 후보(시험 안 함): 출발 칸 탈출용 경유 목표, 진행 검사 시간 늘리기, 실패 뒤 BackUp 복구.
2. **L3 구간 끝 후진 호 여유가 좁음**: 설계 45 Cartographer 판에서 최소 0.0001 m. 계획 쓸림 0.031~0.037 m 가 추종에서 3 cm 줄어듦.

## 실물에서 다시 정할 값(모두 모의 값)
- x_icr(회전 중심, 모의 −0.0658 m): 실물 제자리 회전 시험. rotation_center TF · footprint 가 이 값에 기댐.
- `turn_floor` 의 `w_min` · `move_ff_cmd/act`: 실물 회전 명령별 실제 회전 속도.
- AMCL `initial_pose`: 모의 base_link (0.734, 0.95, π). kanichong 원본은 x 0.69365 — 기준점(base_link 인지 다른 점인지)부터 확인.
- 지도(`/map` · `/map_objects`)는 실물 미로 SLAM 뒤 인지 파트가 localization 패키지에서 바꿈.
- AMCL 매 스캔 갱신의 Jetson 부하(측정 안 함).

## 다른 담당과 엮인 일
- **bringup EKF(kanichong)**: 팀 `bringup/config/ekf.yaml` 은 바퀴 vx 만 받아 제자리 회전 때 base_link 옆 이동(vy = −x_icr · w)을 모름 → AMCL 오차가 커져 loc_monitor SUSPECT 가 남(위 02). ICR 보정 odom(`nav2/nodes/icr_odom.py`: vx 그대로, vy = −x_icr · w_imu) → EKF 가 vx · vy 를 받으면 모의에서 사라짐. 인지 파트가 kanichong 에게 따로 요청함.
- **twist_mux(kanichong · LCY)**: `/localization/lock` lock 줄(01 참고).
- **perception_survey(인지)**: `/localization/status` 를 이미 받음. frame 은 map.
