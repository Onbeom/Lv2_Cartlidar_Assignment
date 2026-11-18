# 브리지 검증 결과 — A′: Lyrical source + empy·lark 링크 (native Lyrical)

결과 폴더: `L1_A2_261025_151244`

이 실행에서 뺀 것: camera_info, scan

## B0 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| ROS_DISTRO 인식 | = lyrical | lyrical | 통과 |
| ROS 관련 오류·심볼 미해결 | 0줄 (libisaacsim.ros2.core.lyrical.so 안내 줄 제외) | 0줄 | 통과 |
| libisaacsim.ros2.core.lyrical.so 없음 안내(jazzy 백엔드 사용) | 기록 | 1줄 | 기록 |
| librcl.so 위치 | '/opt/ros/lyrical/' 안 | 불러오지 않음 | 실패 |
| librmw.so 위치 | '/opt/ros/lyrical/' 안 | 불러오지 않음 | 실패 |
| librmw_cyclonedds_cpp.so 위치 | '/opt/ros/lyrical/' 안 | 불러오지 않음 | 실패 |
| Isaac 백엔드 | 기록 | 없음 | 기록 |
| 내장 jazzy 라이브러리 혼입 | 기록(실패 아님) | 0개 | 기록 |
| ★ Isaac 프로세스 안 rclpy | 기록(실패 아님) | 시스템 rclpy 불러옴 | 기록 |

## B1 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| /clock 수신 | 1개 이상 | 0개 | 실패 |
| 시각이 거꾸로 간 횟수 | 0 | 0 | 실패 |
| 일시정지 시험 | 기록 필요 | pause 기록 또는 /clock 없음 | 실패 |

## B2 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| /left/image_raw 크기·encoding·step·길이·frame_id | 640x480 rgb8 step 1920, camera_left | 0개 중 불일치 0 | 실패 |
| /left/camera_info 크기·fx·fy·cx·cy·frame_id | fx=fy=317.0431 cx=320.0 cy=240.0 (상대 0.0001) | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| left Image와 CameraInfo stamp 일치 | 95% 이상 | 데이터 없음 | 실패 |
| /right/image_raw 크기·encoding·step·길이·frame_id | 640x480 rgb8 step 1920, camera_right | 0개 중 불일치 0 | 실패 |
| /right/camera_info 크기·fx·fy·cx·cy·frame_id | fx=fy=317.0431 cx=320.0 cy=240.0 (상대 0.0001) | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| right Image와 CameraInfo stamp 일치 | 95% 이상 | 데이터 없음 | 실패 |
| ★ /left/camera_info d 길이·r 단위행렬·p | P[3] = 0.0000, 상대 0.0001, 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| ★ /right/camera_info d 길이·r 단위행렬·p | P[3] = -15.8522, 상대 0.0001, 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| /tf world→base_link | 부모 world, 위치 ±0.0001 m, 각도 ±0.0001 rad | 0개 중 불일치 0 | 실패 |
| ★ /tf 변환 2개(base_link·marker_link) | 둘 다 위치 ±0.0001 m, 각도 ±0.0001 rad | marker 0개 중 불일치 0, 같은 메시지에 함께 실림 0회 | 실패 |
| /scan ranges 개수가 각도 설정과 맞음 | 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| /scan 정면 거리 | 2.0 ± 0.02 m, 모든 메시지 | 0개 중 범위 밖 0, 값 없음 0 | 기록(이 실행에서 뺌) |
| /scan frame_id | lidar_link | 불일치 0 | 기록(이 실행에서 뺌) |
| ★ /scan intensities 길이 = ranges 길이(또는 0), 모두 유한 | 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| /odom frame·위치·자세 | odom→base_link, 위치 ±0.001 m, 각도 ±0.0001 rad | 0개 중 불일치 0 | 실패 |
| /odom 속도(첫 메시지) | 넣은 값과 같음(기록) | 없음 | 기록(다름) |
| /odom covariance 길이 | 36, 36 | None, None | 실패 |
| /cmd_vel 수신값 | 보낸 값 [0.25, -0.125, 0.0625, 0.5, -0.75, 1.0]과 차이 ≤ 1e-06 | 바뀐 값 0종, 불일치 0 | 실패 |
| ★ /cmd_vel_stamped(TwistStamped, 범용 구독) 수신값 | 보낸 값 [0.375, -0.0625, 0.03125, -0.25, 0.125, -1.5]·frame base_footprint과 차이 ≤ 1e-06 | 마지막 값 없음, frame None, 출력 속성 0개 | 실패 |

## B3 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| 재생 시간(실제 시간) | ≥ 600 s | 0.0 s (sim 0.0 s, 단계 0) | 실패 |
| /clock 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 실패 |
| /tf 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 실패 |
| /odom 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 실패 |
| /left/image_raw 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 실패 |
| /right/image_raw 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 실패 |
| /left/camera_info 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 기록(이 실행에서 뺌) |
| /right/camera_info 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 기록(이 실행에서 뺌) |
| /scan 받은 개수 | 발행 추정 0의 95% 이상 | 0 (0.0%) | 기록(이 실행에서 뺌) |
| /clock 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 실패 |
| /tf 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 실패 |
| /odom 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 실패 |
| /left/image_raw 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 실패 |
| /right/image_raw 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 실패 |
| /left/camera_info 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 기록(이 실행에서 뺌) |
| /right/camera_info 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 기록(이 실행에서 뺌) |
| /scan 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 기록(이 실행에서 뺌) |
| Isaac Sim 종료 | 정상 종료(0) | 139 | 실패 |
| GPU 메모리 증가 | 1분 대비 5% 초과 시 표시 | 기록 없음 | 기록 |
| Isaac Sim 프로세스 메모리 증가 | 1분 대비 5% 초과 시 표시 | 기록 없음 | 기록 |
