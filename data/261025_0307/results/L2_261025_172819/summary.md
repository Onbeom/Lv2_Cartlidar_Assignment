# L2 로봇 장면 검사 결과 (방식 (a), 조건 A′, 모의 시험)

결과 폴더: `L2_261025_172819`

## C1 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| C1-1 /clock 진행 | 받음, 거꾸로 0회 | 17999개, 거꾸로 0회 | 통과 |
| C1-2 바깥 노드 use_sim_time | robot_state_publisher·관찰 노드 모두 True | /robot_state_publisher Boolean value is: True / /l2_observer Boolean value is: True | 통과 |

## C2 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| C2-1 /clock 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17999 (100.0%) | 통과 |
| C2-1 /odom 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17999 (100.0%) | 통과 |
| C2-1 /joint_states 받은 개수 | 설정 50 Hz × 600 s = 30001의 95 % 이상 | 30001 (100.0%) | 통과 |
| C2-1 /scan_front 받은 개수 | 설정 10 Hz × 600 s = 6000의 95 % 이상 | 5977 (99.6%) | 통과 |
| C2-1 /scan_rear 받은 개수 | 설정 10 Hz × 600 s = 6000의 95 % 이상 | 5983 (99.7%) | 통과 |
| C2-1 /top_camera_front/color/image_raw 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17541 (97.4%) | 통과 |
| C2-1 /top_camera_rear/color/image_raw 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17760 (98.7%) | 통과 |
| C2-1 /arm_camera/image_raw 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17767 (98.7%) | 통과 |
| C2-1 /top_camera_front/color/camera_info 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17996 (100.0%) | 통과 |
| C2-1 /top_camera_rear/color/camera_info 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17996 (100.0%) | 통과 |
| C2-1 /arm_camera/camera_info 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17996 (100.0%) | 통과 |
| C2-1 /top_camera_front/aligned_depth_to_color/image_raw 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17981 (99.9%) | 통과 |
| C2-1 /top_camera_rear/aligned_depth_to_color/image_raw 받은 개수 | 설정 30 Hz × 600 s = 18000의 95 % 이상 | 17995 (100.0%) | 통과 |
| C2-1 /tf odom→base_footprint 받은 개수 | 18000의 95 % 이상 | 17999 (100.0%) | 통과 |
| C2-2 /top_camera_front/color/image_raw 지연 | 100 % ≤ 0.3 s (받은 순간 /clock − stamp) | 최대 0.100 s, p95 0.033 s, 초과 0개 | 통과 |
| C2-2 /top_camera_rear/color/image_raw 지연 | 100 % ≤ 0.3 s (받은 순간 /clock − stamp) | 최대 0.100 s, p95 0.033 s, 초과 0개 | 통과 |
| C2-2 /arm_camera/image_raw 지연 | 100 % ≤ 0.3 s (받은 순간 /clock − stamp) | 최대 0.100 s, p95 0.033 s, 초과 0개 | 통과 |
| C2-2 /top_camera_front/aligned_depth_to_color/image_raw 지연 | 100 % ≤ 0.3 s (받은 순간 /clock − stamp) | 최대 0.100 s, p95 0.033 s, 초과 0개 | 통과 |
| C2-2 /top_camera_rear/aligned_depth_to_color/image_raw 지연 | 100 % ≤ 0.3 s (받은 순간 /clock − stamp) | 최대 0.100 s, p95 0.033 s, 초과 0개 | 통과 |
| C2-3 /scan_front 정면 벽 거리 | 계산값 ± 0.02 m | 측정 1.3145 / 계산 1.3145 m (빔 180) | 통과 |
| C2-3 /scan_front 상자 방향 | 상자에 닿아야 하는 빔(몸체 가림 < 0.3 m 제외)의 80 % 이상이 계산값 ± 0.02 m | 28/28 (100%), 몸체 가림 제외 0/28, 계산 각도 [59, 86]번 빔, 전체 스캔 중 < 0.3 m 빔 66개 | 통과 |
| C2-3 /scan_rear 정면 벽 거리 | 계산값 ± 0.02 m | 측정 0.8145 / 계산 0.8145 m (빔 180) | 통과 |
| C2-3 /scan_rear 상자 방향 | 상자에 닿아야 하는 빔(몸체 가림 < 0.3 m 제외)의 80 % 이상이 계산값 ± 0.02 m | 13/13 (100%), 몸체 가림 제외 12/25, 계산 각도 [273, 297]번 빔, 전체 스캔 중 < 0.3 m 빔 73개 | 통과 |
| C2-4 front 카메라 표식 방향 | 표식이 계산한 쪽 절반에 보임 | 측정 u 176.1 / 계산 u 178.3 (cx 324.4, 차 -2.2 px, 화소 3691) | 통과 |
| C2-4 rear 카메라 표식 방향 | 표식이 계산한 쪽 절반에 보임 | 측정 u 502.7 / 계산 u 500.8 (cx 324.4, 차 +1.9 px, 화소 7631) | 통과 |
| C2-4 앞 카메라 중앙 깊이 | 계산값 ± 0.02 m | 측정 0.8424 / 계산 0.8431 m (floor) | 통과 |
| C2-5 /joint_states 이름·위치 | 이름 11개 순서 일치, 위치 = 참값 ± 0.001 rad | 30001개 중 이름 불일치 0, 참값과 비교 30001개, 최대 차 0.00e+00 | 통과 |
| ★C2-6 front 깊이 형식·짝 | 32FC1 640x480, 컬러와 stamp 차 ≤ 0.02 s ≥ 95 % | 17981개 중 형식 불일치 0, 짝 97.5% | 통과 |
| ★C2-6 rear 깊이 형식·짝 | 32FC1 640x480, 컬러와 stamp 차 ≤ 0.02 s ≥ 95 % | 17995개 중 형식 불일치 0, 짝 98.7% | 통과 |

## C3 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| C3-1 odom → 인지 frame TF 조회 | 10 s마다 6개 frame, 실패 0회 | 360회 중 실패 0 | 통과 |
| C3-2 TF 발행자 하나 | 자식 frame마다 부모 1개, /tf = Isaac + robot_state_publisher, /tf_static = robot_state_publisher, base_footprint의 부모 = odom | 부모 여럿 없음, /tf ['_L2Phys_PubTF', 'robot_state_publisher'], /tf_static ['robot_state_publisher'], base_footprint 부모 ['odom'] | 통과 |

## C4 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| C4-1 front 컬러·CameraInfo stamp | ≥ 95 % 같음 | 17541/17541 (100.0%) | 통과 |
| C4-2 front CameraInfo 값 | K = [605.851, 605.68, 324.366, 245.139](상대 0.0001), k[8]=1·k[1]=k[3]=0·fx,fy>0, R 단위, P=[K|0], frame·크기 | 17996개 중 불일치 0 | 통과 |
| C4-1 rear 컬러·CameraInfo stamp | ≥ 95 % 같음 | 17760/17760 (100.0%) | 통과 |
| C4-2 rear CameraInfo 값 | K = [605.851, 605.68, 324.366, 245.139](상대 0.0001), k[8]=1·k[1]=k[3]=0·fx,fy>0, R 단위, P=[K|0], frame·크기 | 17996개 중 불일치 0 | 통과 |
| C4-1 arm 컬러·CameraInfo stamp | ≥ 95 % 같음 | 17767/17767 (100.0%) | 통과 |
| C4-2 arm CameraInfo 값 | K = [283.0, 283.0, 320.0, 240.0](상대 0.0001), k[8]=1·k[1]=k[3]=0·fx,fy>0, R 단위, P=[K|0], frame·크기 | 17996개 중 불일치 0 | 통과 |

## C5 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| C5-1 명령 → 바퀴 목표 | 기구학 계산값과 차이 ≤ 1e-06 rad/s, 받은 명령 = 보낸 값 | 30001행 최대 차 0.00e+00(끊김 경계 0행 제외), 보냄 300·받음 300, 값·frame 불일치 0 | 통과 |
| C5-2 직진 0.10 m/s 5 s | 이동 0.50 ± 0.025 m, 옆 ≤ 0.02 m, yaw ≤ 2° | 이동 0.5000 m, 옆 +0.0001 m, yaw -0.01° | 통과 |
| C5-3 회전 +0.5 rad/s 4 s | yaw 증가(반시계), 회전량은 기록 | +102.1° (명령 114.6°, 비율 0.89) | 통과 |
| C5-4 정지 | 0 명령을 받은 뒤 0.5 s 안에 < 0.01 m/s(직진·회전 뒤), 명령이 끊기면 0.5 s 뒤 바퀴 목표 0, 그 뒤 0.5 s 안에 정지 | 직진 뒤 0 명령 15.017 s → 0.050 s, 회전 뒤 0 명령 21.017 s → 0.008 s (그 0.5 s 뒤 ωz -0.0007 rad/s), 마지막 명령 24.950 s → 목표 0 25.467 s → 정지 25.500 s | 통과 |

## 간격 — 기록

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| ★/arm_camera/camera_info 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 33 ms, 받은 시각 194 ms | 기록 |
| ★/arm_camera/image_raw 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 167 ms, 받은 시각 387 ms | 기록 |
| ★/clock 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 33 ms, 받은 시각 166 ms | 기록 |
| ★/joint_states 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 25 ms, 받은 시각 175 ms | 기록 |
| ★/odom 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 33 ms, 받은 시각 166 ms | 기록 |
| ★/scan_front 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 200 ms, 받은 시각 470 ms | 기록 |
| ★/scan_rear 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 200 ms, 받은 시각 381 ms | 기록 |
| ★/top_camera_front/aligned_depth_to_color/image_raw 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 67 ms, 받은 시각 268 ms | 기록 |
| ★/top_camera_front/color/camera_info 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 33 ms, 받은 시각 194 ms | 기록 |
| ★/top_camera_front/color/image_raw 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 533 ms, 받은 시각 909 ms | 표시 |
| ★/top_camera_rear/aligned_depth_to_color/image_raw 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 67 ms, 받은 시각 205 ms | 기록 |
| ★/top_camera_rear/color/camera_info 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 33 ms, 받은 시각 194 ms | 기록 |
| ★/top_camera_rear/color/image_raw 최대 간격 | 기록, stamp 간격 0.5 s 초과 시 표시 | stamp 167 ms, 받은 시각 381 ms | 기록 |

## 전체 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| 시뮬레이션 시간 | ≥ 600 s | 600.0 s (실제 595 s) | 통과 |
| Isaac Sim 종료 | 정상 종료(0) | 0 | 통과 |
