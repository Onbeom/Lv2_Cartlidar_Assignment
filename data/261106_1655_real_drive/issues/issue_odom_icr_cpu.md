제목: [bringup · EKF] odom_icr.py 가 바퀴가 멈춰 있어도 CPU 코어 하나의 27~28 % 를 씀 (Jetson 실물)

## 무엇을 봤나
Jetson 정지 상태 전체 bringup(`use_planner:=false`, 바퀴 막고 움직임 명령 없음)에서, 측정 구간 5 분 동안의 프로세스별 평균 CPU 입니다(/proc 앞뒤 차이, 코어 하나 = 100).

| 측정 | odom_icr | 참고: EKF | 참고: robot_state_publisher |
|---|---|---|---|
| 11-13 18:00 | **27.8** | 13.1 | 4.1 |
| 11-13 18:07 | **26.6** | 14.6 | 6.7 |

- odom_icr 는 `/odom` + `/imu/data` 를 받아 vy 한 칸을 채워 `/odom_icr` 로 다시 냅니다(#92). 하는 일에 비해 EKF 의 두 배쯤 씁니다.
- 같은 측정에서 전체 CPU 평균은 84.9 % · 91.9 % 였고, 인지 결과가 300 ms 를 넘는 비율이 12.8 % · 58.1 % 였습니다(부하 전체는 별도 Issue).

## 왜 그런지 (코드를 읽고 추정, 실측 아님)
- Python 노드가 들어오는 메시지마다 콜백을 돕니다: `/imu/data` 는 imu_sensor_broadcaster 가 controller_manager 주기(`update_rate: 100`)로, `/odom` 은 diff_drive_controller `publish_rate` 를 설정하지 않아 기본값으로 냅니다.
- `Odometry` 는 공분산 배열 두 개(36 칸씩)를 가져서, Python 에서 받고 · 고치고 · 다시 내는 비용이 큽니다.

## 요청 (하나를 골라 주세요)
1. odom_icr 를 C++ 로 옮기기 (가능하면 base 컨테이너 안 composable 노드로).
2. 또는 `/odom` 을 EKF 주기(50 Hz)에 맞추고, IMU 는 필요한 각속도만 쓰도록 주기를 낮추기.

## 한계
- 로봇이 미로 밖이라 위치 추정은 실패한 상태였습니다(odom_icr 와는 무관하다고 봄).
- `/odom` · `/imu/data` 의 실제 발행 주기는 재지 않았습니다(설정 값만 봄).
- 결과: Jetson `~/real_drive/p7a_out/1113_1800`, `1113_1807` 의 summary.txt '프로세스별 CPU'.
