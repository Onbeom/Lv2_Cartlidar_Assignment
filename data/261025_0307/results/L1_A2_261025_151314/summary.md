# 브리지 검증 결과 — A′: Lyrical source + empy·lark 링크 (native Lyrical)

결과 폴더: `L1_A2_261025_151314`

이 실행에서 뺀 것: scan, stamped

## B0 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| ROS_DISTRO 인식 | = lyrical | lyrical | 통과 |
| ROS 관련 오류·심볼 미해결 | 0줄 (libisaacsim.ros2.core.lyrical.so 안내 줄 제외) | 0줄 | 통과 |
| libisaacsim.ros2.core.lyrical.so 없음 안내(jazzy 백엔드 사용) | 기록 | 1줄 | 기록 |
| librcl.so 위치 | '/opt/ros/lyrical/' 안 | /opt/ros/lyrical/lib/librcl.so | 통과 |
| librmw.so 위치 | '/opt/ros/lyrical/' 안 | /opt/ros/lyrical/lib/librmw.so | 통과 |
| librmw_cyclonedds_cpp.so 위치 | '/opt/ros/lyrical/' 안 | /opt/ros/lyrical/lib/librmw_cyclonedds_cpp.so | 통과 |
| Isaac 백엔드 | 기록 | libisaacsim.ros2.core.jazzy.so | 기록 |
| 내장 jazzy 라이브러리 혼입 | 기록(실패 아님) | 0개 | 기록 |
| ★ Isaac 프로세스 안 rclpy | 기록(실패 아님) | 시스템 rclpy 불러옴 | 기록 |

## B1 — 통과

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| /clock 수신 | 1개 이상 | 35997개 | 통과 |
| 시각이 거꾸로 간 횟수 | 0 | 0 | 통과 |
| 일시정지 중 시각 증가 | 없음 | 메시지 0개, 증가 없음 | 통과 |
| 재생 재개 후 시각 증가 | 있음 | 있음 | 통과 |

## B2 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| /left/image_raw 크기·encoding·step·길이·frame_id | 640x480 rgb8 step 1920, camera_left | 18105개 중 불일치 0 | 통과 |
| /left/camera_info 크기·fx·fy·cx·cy·frame_id | fx=fy=317.0431 cx=320.0 cy=240.0 (상대 0.0001) | 18105개 중 불일치 18105 (첫 값 fx 0.0 cx 0.0 fy 240.0 cy 0.0) | 실패 |
| left Image와 CameraInfo stamp 일치 | 겹치는 구간 영상의 95% 이상 | 17996/17996 (100.0%) | 통과 |
| /right/image_raw 크기·encoding·step·길이·frame_id | 640x480 rgb8 step 1920, camera_right | 18159개 중 불일치 0 | 통과 |
| /right/camera_info 크기·fx·fy·cx·cy·frame_id | fx=fy=317.0431 cx=320.0 cy=240.0 (상대 0.0001) | 18105개 중 불일치 18105 (첫 값 fx 0.0 cx 0.0 fy 240.0 cy 0.0) | 실패 |
| right Image와 CameraInfo stamp 일치 | 겹치는 구간 영상의 95% 이상 | 17996/17996 (100.0%) | 통과 |
| ★ /left/camera_info d 길이·r 단위행렬·p | P[3] = 0.0000, 상대 0.0001, 모든 메시지 | 18105개 중 불일치 18105 (예: r [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 317.0431]; p [0.0, 320.0, 0.0, 0.0, 317.0431, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]) | 실패 |
| ★ /right/camera_info d 길이·r 단위행렬·p | P[3] = -15.8522, 상대 0.0001, 모든 메시지 | 18105개 중 불일치 18105 (예: r [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 317.0431]; p [0.0, 320.0, -15.8522, 0.0, 317.0431, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]) | 실패 |
| /tf world→base_link | 부모 world, 위치 ±0.0001 m, 각도 ±0.0001 rad | 35997개 중 불일치 0 | 통과 |
| ★ /tf 변환 2개(base_link·marker_link) | 둘 다 위치 ±0.0001 m, 각도 ±0.0001 rad | marker 35997개 중 불일치 0, 같은 메시지에 함께 실림 35997회 | 통과 |
| /scan ranges 개수가 각도 설정과 맞음 | 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| /scan 정면 거리 | 2.0 ± 0.02 m, 모든 메시지 | 0개 중 범위 밖 0, 값 없음 0 | 기록(이 실행에서 뺌) |
| /scan frame_id | lidar_link | 불일치 0 | 기록(이 실행에서 뺌) |
| ★ /scan intensities 길이 = ranges 길이(또는 0), 모두 유한 | 모든 메시지 | 0개 중 불일치 0 | 기록(이 실행에서 뺌) |
| /odom frame·위치·자세 | odom→base_link, 위치 ±0.001 m, 각도 ±0.0001 rad | 35997개 중 불일치 0 | 통과 |
| /odom 속도(첫 메시지) | 넣은 값과 같음(기록) | [0.25, -0.125, 0.0625] [0.5, -0.25, 0.125] | 기록 |
| /odom covariance 길이 | 36, 36 | 36, 36 | 통과 |
| /cmd_vel 수신값 | 보낸 값 [0.25, -0.125, 0.0625, 0.5, -0.75, 1.0]과 차이 ≤ 1e-06 | 바뀐 값 1종, 불일치 0 | 통과 |
| ★ /cmd_vel_stamped(TwistStamped, 범용 구독) 수신값 | 이 실행에서는 뺌(--skip, 별도 실행으로 판정) | - | 기록 |

## B3 — 실패

| 항목 | 기준 | 측정 | 판정 |
|---|---|---|---|
| 재생 시간(실제 시간) | ≥ 600 s | 599.8 s (sim 600.0 s, 단계 35998) | 통과 |
| /clock 받은 개수 | 발행 추정 35998의 95% 이상 | 35997 (100.0%) | 통과 |
| /tf 받은 개수 | 발행 추정 35998의 95% 이상 | 35997 (100.0%) | 통과 |
| /odom 받은 개수 | 발행 추정 35998의 95% 이상 | 35997 (100.0%) | 통과 |
| /left/image_raw 받은 개수 | 발행 추정 17999의 95% 이상 | 18105 (100.6%) | 통과 |
| /right/image_raw 받은 개수 | 발행 추정 17999의 95% 이상 | 18159 (100.9%) | 통과 |
| /left/camera_info 받은 개수 | 발행 추정 17999의 95% 이상 | 18105 (100.6%) | 통과 |
| /right/camera_info 받은 개수 | 발행 추정 17999의 95% 이상 | 18105 (100.6%) | 통과 |
| /scan 받은 개수 | 발행 추정 6000의 95% 이상 | 0 (0.0%) | 기록(이 실행에서 뺌) |
| /clock 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 630회 (평소 16.2 ms, 최대 74.7 ms, 평균 59.4 Hz) | 실패 |
| /tf 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 631회 (평소 15.8 ms, 최대 75.8 ms, 평균 59.4 Hz) | 실패 |
| /odom 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 631회 (평소 16.0 ms, 최대 76.1 ms, 평균 59.4 Hz) | 실패 |
| /left/image_raw 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 19회 (평소 33.2 ms, 최대 147.3 ms, 평균 29.7 Hz) | 실패 |
| /right/image_raw 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 19회 (평소 33.2 ms, 최대 147.6 ms, 평균 29.7 Hz) | 실패 |
| /left/camera_info 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 19회 (평소 33.3 ms, 최대 147.1 ms, 평균 29.7 Hz) | 실패 |
| /right/camera_info 끊김 | 평소 간격 3배 초과 0회 (일시정지 구간 제외) | 19회 (평소 33.3 ms, 최대 147.4 ms, 평균 29.7 Hz) | 실패 |
| /scan 끊김 | 평소 간격 3배 초과 0회 | 데이터 부족 | 기록(이 실행에서 뺌) |
| Isaac Sim 종료 | 정상 종료(0) | 0 | 통과 |
| GPU 메모리 증가 | 1분 대비 5% 초과 시 표시 | 2055 → 2044 MiB (-0.5%) | 기록 |
| Isaac Sim 프로세스 메모리 증가 | 1분 대비 5% 초과 시 표시 | 7041 → 7043 MiB (+0.0%) | 기록 |

## 발행자 QoS (관찰 노드가 본 값)

| 토픽 | 노드 | reliability | durability | depth |
|---|---|---|---|---|
| /clock | _BridgeCheck_PublishClock | 1 | 2 | 10 |
| /tf | _BridgeCheck_PublishTF | 1 | 2 | 10 |
| /odom | _BridgeCheck_PublishOdom | 1 | 2 | 10 |
| /left/image_raw | _Render_PostProcess_SDGPipeline_Replicator_01_NodeWriterWriter_01 | 1 | 2 | 10 |
| /right/image_raw | _Render_PostProcess_SDGPipeline_Replicator_NodeWriterWriter | 1 | 2 | 10 |
| /left/camera_info | _Render_PostProcess_SDGPipeline_Replicator_01_NodeWriterWriter | 1 | 2 | 10 |
| /right/camera_info | _Render_PostProcess_SDGPipeline_Replicator_NodeWriterWriter_01 | 1 | 2 | 10 |
