## 요청 (bringup, 작성자 kanichong)
`bringup/config/ekf.yaml`의 바퀴 odom 입력에 스키드 회전 중심(ICR) 옆 이동 보정을 넣어 주세요.

## 왜
- 지금 EKF는 바퀴 vx와 IMU yaw rate만 받습니다. 4륜 스키드는 회전 중심이 base_link 뒤(모의 x_icr −0.0658 m)에 있어서, 제자리 회전 때 base_link가 옆으로 vy = −x_icr · w만큼 움직입니다. EKF는 이 움직임을 모릅니다.
- Isaac 모의 K0(같은 길, 목표 7개)에서 비교한 결과입니다.

| EKF | AMCL RMSE · 최대 | Cartographer RMSE | 위치 상태 SUSPECT |
|---|---|---|---|
| 팀 ekf.yaml | 0.0221 · 0.051 m | 0.0147 m | 2 · 1번 |
| ICR 보정 | 0.0157 · 0.028 m | 0.0083 m | 0 · 0번 |

## 제안
- 작은 노드가 `/odom`(vx)과 `/imu/data`(w)를 받아 vy = −x_icr · w를 채운 `/odom_icr`를 냅니다. EKF는 `odom0: /odom_icr`로 받고, `odom0_config`에서 vx와 vy를 켭니다.
- 모의에서 쓴 노드와 설정이 있으니 필요하면 드리겠습니다.
- x_icr은 모의 값입니다. 실물 제자리 회전 시험으로 다시 재야 합니다.
