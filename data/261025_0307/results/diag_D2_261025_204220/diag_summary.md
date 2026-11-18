# 받는 쪽 영상 손실 진단 결과 (검증계획 12장, 모의)

## diag_D2_261025_204220 (rmem_max 67108864, 실행 중 RcvbufErrors 증가 0.0)

| 수신기 | 토픽 | 받은 개수 | 손실 사건(> 0.1 s) | 잃은 장 수(추정) | 최대 간격 |
|---|---|---|---|---|---|
| R1 Python 관찰 노드 | /top_camera_front/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R1 Python 관찰 노드 | /top_camera_rear/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R1 Python 관찰 노드 | /arm_camera/image_raw | 17996 | 0 | 0 | 33 ms |
| R1 Python 관찰 노드 | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R1 Python 관찰 노드 | /top_camera_rear/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /top_camera_front/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /top_camera_rear/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /arm_camera/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /top_camera_rear/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /top_camera_front/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /top_camera_rear/color/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /arm_camera/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /top_camera_rear/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |

| 수신기 | CPU 평균·최대 (%) | 사건 중 RcvbufErrors 증가와 겹친 것 |
|---|---|---|
| R1 Python 관찰 노드 | 41.1 / 54.4 | 0/0 |
| R2 최소 Python | 26.9 / 41.8 | 0/0 |
| R3 최소 C++ | 7.0 / 10.9 | 0/0 |

| 판정 | 기준 | 측정 | 결과 |
|---|---|---|---|
| E2 Python 처리 원인 | R1만 손실, R2·R3 0 | R1 0, R2 0, R3 0 | 판단 불가(손실 없음) |
| E3 C++ 수신 영향 | R3 손실 사건 수·최대 간격 | R3 사건 0, 최대 간격 33 ms | 기록 |
| E4 송신 쪽 원인 | 세 수신기가 같은 시점에 잃고 RcvbufErrors 증가 0 | 사건 중 RcvbufErrors 증가와 겹친 것 0/0 | 판단 불가(손실 없음) |
| E1 수신 버퍼(이 조건) | 손실 시점 ±1 s에 RcvbufErrors 증가 | 0/0, 실행 중 증가 합 0.0 | 판단 불가(손실 없음) |
