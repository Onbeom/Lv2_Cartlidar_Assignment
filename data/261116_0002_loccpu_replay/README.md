# loc_cpp 상태 출력 비교 (PC, 11-16 00:0x ~ 00:45)
- 목적: 위치 추정 C++ 판(perception 패키지 scan_body_mask · loc_scan_merge · loc_monitor)이 Python 판과 같은 상태를 내는지 (p7loc 판정 뒤 PR 전 조건, README '다음 측정 기준').
- 입력: 11-14 미로 bag (maze_1114_1317) 에서 /scan_front · /scan_rear · /odom · /imu/data · /tf_static(라이다 요 #304 값으로) + /odom 으로 만든 odom → base_link /tf (make_replay_bag.py → bag_replay).
- 방법: dual.launch.py — 같은 재생 · 같은 AMCL 에 Python 판(원래 토픽, 다시 찾기 /initialpose 가 실제로 AMCL 에 닿음)과 C++ 판(/cpp/… 로 이름 바꿈, /cpp/initialpose 는 AMCL 에 안 닿음)을 함께 띄움. run.sh (따로 도메인 89 · LOCALHOST, bag 재생 1 배), compare.py.
- 1 · 2 차는 무효: 재생기의 /tf_static QoS 가 노드와 안 맞아 정적 TF 를 못 받아 스캔이 하나도 안 나옴(입력 없음 → 15 s 에 LOST). 3 차에 play_qos.yaml(transient_local) 로 고침.

## 3 차 결과 (out/cmp)
| 항목 | Python | C++ | 비교 |
|---|---|---|---|
| 상태 바뀜 | 19 번 | 19 번 | 값 순서 같음 (OK → SUSPECT → LOST → RECOVERING → CONFIRM → OK … → FAILED), 바뀐 시각 차 0 ~ 1.04 s (다시 찾기 끝 CONFIRM · 그 뒤 LOST 에서만 차이, 나머지 0.00 ~ 0.01 s) |
| lock 바뀜 | 4 번 | 4 번 | 값 · 시각 같음 (차 0.01 s 안) |
| 다시 찾기 /initialpose | 4 번 (4.5 · 15.2 · 26.2 · 66.5 s) | 4 번 (3.8 · 15.1 · 25.2 · 67.2 s) | 횟수 같음, 시각 차 −1.0 ~ +0.7 s |
| /scan_front_filtered | 5549 | 5549 | 100 % 같음 |
| /scan_rear_filtered | 5550 | 5550 | 100 % 같음 |
| /scan_merged | 5548 | 5548 | 98.4 % 같음. 다른 86 장 중 84 장은 Python 쪽이 C++ 쪽 점의 부분집합 (Python 이 뒤 스캔을 못 붙인 장, 11-14 에 본 '뒤 스캔 못 붙임'), 2 장은 붙인 뒤 스캔이 다름 |

- 판단: 상태 · lock · 다시 찾기 횟수 같음, 차체 가림 스캔 완전히 같음. 시각 차 1 s 안팎은 다시 찾기 탐색 시간(실행 속도) 차이로 봄.
- 한계: 이 bag 은 11-14 의 틀어진 녹화(IMU 영점 · 바퀴 회전)라 위치가 자주 잃고 77 s 에 FAILED → 그 뒤는 비교 못 함. 정상 주행 bag(재녹화 maze_1116) 로 한 번 더 볼 것.
