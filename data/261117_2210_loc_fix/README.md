# 위치 추정 C++ 두 가지 고침 확인 (11-17, PC, 실물 bag 재생 = 모의 재생)

고친 것 (개인 저장소 `~/git/Lv2_Cartlidar_Assignment` dev, 커밋 전):
1. loc_monitor: FAILED 가 끝 상태였음 → `failed_retry_s`(5 s) 간격으로 계속 탐색, 운용자 `/initialpose`(RViz 2D Pose Estimate)를 받으면 CONFIRM 으로.
2. loc_scan_merge: 앞 스캔이 올 때만 /scan_merged 를 냄 → 앞 스캔이 `front_timeout`(0.5 s) 넘게 없으면 뒤 스캔만으로 냄.

입력: 11-14 미로 bag 재생본 `261116_0002_loccpu_replay/bag_replay` (556 s, 이 bag 은 77 s 쯤 FAILED 가 났음 — 같은 README).

| 시험 | 값 | 예상 (실행 전) |
|---|---|---|
| A. 그대로 재생 | FAILED 뒤 사건 | `failed_retry` 1 번, 그 뒤 search_global 이 약 5 s(벽시계) 간격으로 계속 — 예전엔 FAILED 뒤 탐색 0 번 |
| A. 그대로 재생 | 수동 /initialpose 직후 상태 | FAILED → CONFIRM (`why: manual_initialpose`) 1 초 안 — 예전엔 FAILED 그대로 |
| B. 앞 스캔 뺀 재생 (60 s) | /scan_merged 장수 | 뒤 스캔 수에 가까움(약 10 Hz × 60 s ≈ 600 장, 시작 0.5 s 빼고) — 예전엔 0 장 |
| B | 경고 줄 | '뒤 스캔만으로' 1 번 |

## 결과 (11-18 09:4x ~ 09:54, PC 재생 — 실물 bag 이지만 로봇 없이 돌린 모의 재생)
| 시험 | 결과 | 판단 |
|---|---|---|
| A. FAILED 뒤 | `FAILED`(recover_s 62.36) → `failed_retry`(5 s) → `CONFIRM` → `OK`(recover_s 65.52) | 통과 — 예전엔 FAILED 에서 멈춤 |
| A. 수동 /initialpose | 그때 상태 RECOVERING → `CONFIRM`(`why: manual_initialpose`, pose 0.694 · 0.95 · π) | 통과 (FAILED 상태에서 넣은 것은 아님) |
| B. 앞 스캔 뺀 60 s | 경고 '뒤 스캔만으로' 1 번, /scan_merged 10.02 ~ 10.04 Hz (hz 58 번) | 통과 — 예전 코드는 앞 스캔이 없으면 0 장 |
- 이 bag 은 바퀴 · IMU 가 틀어진 11-14 녹화라 A 에서 LOST 가 자주 남(위치 정확도 판정 아님, 상태 전이만 봄).
- 1 차(11-17 22 시)는 localization 이 빌드 안 된 작업 트리를 써서 무효, out_A · out_B 는 11-18 재실행 결과.
