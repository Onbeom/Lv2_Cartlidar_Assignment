[nav2] Jetson 에서 bt_navigator 활성화가 자주 실패 (7 판 중 6 판 lifecycle 미완) · Nav2 가 다 뜨면 CPU 약 +10 점

## 무엇을 봤나 (실물 Jetson, 정지 상태 5 분, 같은 bringup 인자)
| 판 | Nav2 (`Managed nodes are active` / 실패 노드) | CPU 평균 | 인지 결과 |
|---|---|---|---|
| 11-14 02:34 | 모두 활성 | **92.0 %** | 2.9 Hz |
| 11-14 03:05 | planner_server 실패, controller_server 로그 869 줄 | 91.8 % | 4.4 Hz |
| 11-14 02:49 · 02:58 · 03:12, 11-13 22:06 | **bt_navigator 활성화 실패** (Aborting bringup) | 80.4 ~ 83.1 % | 7.8 ~ 9.5 Hz |
| 11-14 00:03 | bt_navigator 활성화 실패 | 93.4 % | 3.4 Hz (측정 끝 50 s 가 다른 bringup 과 겹쳐 참고만) |
- 7 판 중 6 판에서 Nav2 lifecycle 이 끝까지 안 감(bt_navigator 5 · planner_server 1).
- 다 뜬 판(02:34)에서 bt_navigator · controller_server · planner_server · behavior_server CPU 가 bt_navigator 실패 판(02:49 · 02:58)의 1.5~3.4 배(bt_navigator 13.8 vs 4.1 · controller 13.3 vs 8.7 · planner 11.0 vs 6.0 · behavior 10.7 vs 5.6, 네 서버 합 약 49 vs 24, 코어 하나 = 100, 5 분 평균).

## 부탁
- bt_navigator 활성화 실패 원인 확인(로그: `Failed to change state for node: bt_navigator`). 시작 부하 때 시간 초과면 lifecycle_manager `bond_timeout` · 활성화 순서 조정.
- 정지 상태(목표 없음)에서 Nav2 서버들이 쓰는 CPU 를 줄일 수 있는지(costmap 갱신 주기 · controller 주기 등). 지금 미션 때는 모두 떠야 하므로 다 뜬 상태(약 92 %)가 실제 부하로 봅니다.

## 한계
- 로봇이 미로 밖이라 AMCL 이 위치를 못 잡은 상태(LOST)였음 — 그 영향일 수도 있음. 기록: Jetson `~/real_drive/p7a_out/1114_*`, #265.
