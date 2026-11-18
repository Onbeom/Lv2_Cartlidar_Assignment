@developlcy-oss `split_control:=true` 로 쟀습니다 (2026-11-15 19:42, 실물 Jetson, 설치본 fd6bcbf — #299 포함).

## 조건
11-14 03:12 와 같게: 정지 상태 5 분, 인지 모드 7, 계획기 켬 (`use_planner:=true`), 덧씌우기 없음. 바뀐 것은 `split_control:=true` 하나. 이번에도 bt_navigator 활성화 실패 판 (03:12 와 같은 Nav2 상태).

## 결과 (측정 구간 305 s 평균, 코어 하나 = 100)
| 프로세스 | 담긴 노드 | CPU |
|---|---|---|
| control_manipulator | manipulator_node | **22.2** |
| control_drive | drive_node | 14.7 |
| control_mission | mission_node | 8.1 |
| control_tfmini | tfmini_node · obstacle_stop | 5.1 |
| 넷 합 | | 50.1 |

- 03:12 (한 프로세스) control_container 45.8 → 나눈 뒤 넷 합 50.1 (+4.3). 프로세스 사이 통신 · 프로세스마다 실행기가 늘어난 몫으로 봅니다 (측정 사이 변동 안일 수 있음).
- 전체: CPU 평균 82.0 % (03:12 82.8 %), 프로세스 합 462 (03:12 471). 인지 결과 9.7 Hz · >300 ms 11.0 % (03:12 9.2 Hz · 11.6 %).
- 한 번 잰 값입니다. 기록: Jetson `~/real_drive/p7a_out/1115_1942/summary.txt`.

manipulator 몫이 가장 커서 말씀하신 `controller_state` 구독을 E-stop 감시 때만 켜는 것부터 보시면 될 것 같습니다.
