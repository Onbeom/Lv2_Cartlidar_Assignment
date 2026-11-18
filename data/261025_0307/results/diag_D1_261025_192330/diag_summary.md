# 받는 쪽 영상 손실 진단 결과 (검증계획 12장, 모의)

## diag_D1_261025_192330 (rmem_max 4194304, 실행 중 RcvbufErrors 증가 38878.0)

| 수신기 | 토픽 | 받은 개수 | 손실 사건(> 0.1 s) | 잃은 장 수(추정) | 최대 간격 |
|---|---|---|---|---|---|
| R1 Python 관찰 노드 | /top_camera_front/color/image_raw | 17850 | 6 | 13 | 133 ms |
| R1 Python 관찰 노드 | /top_camera_rear/color/image_raw | 17988 | 0 | 0 | 67 ms |
| R1 Python 관찰 노드 | /arm_camera/image_raw | 17681 | 20 | 52 | 167 ms |
| R1 Python 관찰 노드 | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R1 Python 관찰 노드 | /top_camera_rear/aligned_depth_to_color/image_raw | 17764 | 11 | 22 | 100 ms |
| R2 최소 Python | /top_camera_front/color/image_raw | 17840 | 10 | 35 | 300 ms |
| R2 최소 Python | /top_camera_rear/color/image_raw | 17980 | 0 | 0 | 67 ms |
| R2 최소 Python | /arm_camera/image_raw | 17667 | 31 | 83 | 300 ms |
| R2 최소 Python | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R2 최소 Python | /top_camera_rear/aligned_depth_to_color/image_raw | 17830 | 10 | 20 | 100 ms |
| R3 최소 C++ | /top_camera_front/color/image_raw | 17862 | 9 | 20 | 133 ms |
| R3 최소 C++ | /top_camera_rear/color/image_raw | 17967 | 0 | 0 | 67 ms |
| R3 최소 C++ | /arm_camera/image_raw | 17602 | 44 | 113 | 200 ms |
| R3 최소 C++ | /top_camera_front/aligned_depth_to_color/image_raw | 17996 | 0 | 0 | 33 ms |
| R3 최소 C++ | /top_camera_rear/aligned_depth_to_color/image_raw | 17806 | 17 | 36 | 133 ms |

| 수신기 | CPU 평균·최대 (%) | 사건 중 RcvbufErrors 증가와 겹친 것 |
|---|---|---|
| R1 Python 관찰 노드 | 39.3 / 52.6 | 37/37 |
| R2 최소 Python | 25.8 / 38.8 | 51/51 |
| R3 최소 C++ | 7.1 / 11.8 | 70/70 |

| 판정 | 기준 | 측정 | 결과 |
|---|---|---|---|
| E2 Python 처리 원인 | R1만 손실, R2·R3 0 | R1 37, R2 51, R3 70 | 해당 없음 |
| E3 C++ 수신 영향 | R3 손실 사건 수·최대 간격 | R3 사건 70, 최대 간격 200 ms | 기록 |
| E4 송신 쪽 원인 | 세 수신기가 같은 시점에 잃고 RcvbufErrors 증가 0 | 사건 중 RcvbufErrors 증가와 겹친 것 158/158 | 해당 없음 |
| E1 수신 버퍼(이 조건) | 손실 시점 ±1 s에 RcvbufErrors 증가 | 158/158, 실행 중 증가 합 38878.0 | D2와 비교해 판정 |

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

## 조건 비교 (E1 수신 버퍼 원인: D1에서 RcvbufErrors와 함께 잃고, D2에서 손실·증가 0)

| 폴더 | rmem_max | 손실 사건 R1/R2/R3 | RcvbufErrors 증가 |
|---|---|---|---|
| diag_D1_261025_192330 | 4194304 | 37/51/70 | 38878.0 |
| diag_D2_261025_204220 | 67108864 | 0/0/0 | 0.0 |
