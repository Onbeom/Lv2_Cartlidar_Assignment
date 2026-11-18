# 261103_1909_cam_timing — A-2 영상 시각 측정(모의)

## 무엇 · 왜
- 인지 세션('인지 코드 1102 1')이 11-02 에 요청한 두 값을 **카메라가 켜진 실행**으로 잰다. 지금까지 보낸 값은 영상 토픽을 녹화하지 않은 판의 TF 나이뿐이었다.
  - 조건 1: 인지는 영상 stamp 에 odom → base_footprint 를 조회한다(lookup_near). EKF odom → base_link 최신 stamp 가 영상 stamp 보다 0.1 s 넘게 뒤지면 base 기준으로 바뀐다.
  - TF 나이: map → 카메라 TimePointZero 조회 나이 p95 ≤ 0.05 s.
- 구성 = 팀 저장소 localization 패키지(`localization:=amcl`, 모의라서 `use_amcl_node:=true`) + Nav2 설계 45(`261103_1502_loc_pkg/launch/test_stack.launch.py`). EKF = 팀 `bringup/config/ekf.yaml`(50 Hz, 바퀴 vx + IMU).
  map → odom 은 AMCL(tf_broadcast, transform_tolerance 0.3 s 로 stamp 를 앞당김).
- 장면 `261102_1450_isaac_B_probe/scene_live.py`(B안, `--occluder_mode analytic`), K0 판(강제 이동 없음, 목표 7 개). 인자는 `261103_1502_loc_pkg/run_k01_pkg.sh` 의 K0 amcl 과 같다.
- 모의 장면의 카메라는 **앞 D435 적외선 두 장(infra1 · infra2, 640×480 mono8, 렌더 30 Hz)** 뿐이다. 영상 stamp = 렌더한 모의 시각. 컬러 · 뒤 카메라 · 팔 카메라는 이 장면에 없다(시각 처리 경로는 같다고 보고, 실물 값은 Jetson 지연 측정 때 잰다).

## 파일
- `run_a2_all.sh` → `run_a2.sh`(loc_pkg `run_live_pkg.sh` 사본 + 탐침 3 개). 원본 launch · 설정은 불러 쓰기만 한다.
- `tools/img_lag_probe.py`: infra1 영상을 실제로 구독해 받을 때마다 EKF 최신 stamp · map → 영상 frame 조회 · 영상 stamp 의 odom → base_footprint 조회를 CSV 로 남긴다.
- `tools/evaluate_a2.py`: 채점 → `results/<판>/evaluate_a2.json`. 이전 `261101_0016_isaac_live/evaluate_live.py` S3 · S4 와 같은 계산(프레임만 map).
- 탐침 `261031_0335_slam_launch/tools/tf_probe.py`(10 Hz, map → 앞 카메라 infra1 optical · map → base_link).

## 판정 기준(이전 S3 · S4 그대로, 내가 정함)
- T1: 영상 stamp − 그때 받은 EKF odom → base_link 최신 stamp, **0.1 s 넘는 것 0**. ① bag camera_info 수신 시각(infra1 · infra2, 이전 방식) ② 영상 구독 탐침 — 둘 다.
- T2: map → 앞 카메라 TimePointZero 조회 **≥ 99 %**, 나이(지금 − 돌려받은 stamp) **p95 ≤ 0.05 s**.
- 채점 구간 = map → odom 이 처음 들어온 뒤 0.5 s ~ TF 기록 끝.
- 도구 확인(판정 아님): 영상이 실제로 오는지(탐침 행 수, camera_info 받은 수 ≥ 장면이 낸 수의 99 %).

## 결과 표

| 항목 | 값(단위) | 예상 | 결과 |
|---|---|---|---|
| T1 영상 stamp − EKF 최신 | 중앙 · p95 · 최대(s), 0.1 s 넘음 | 통과. 중앙 ≈ 0.017, 최대 ≤ 0.04(11-01 1단계 판 중앙 0.017 · 최대 0.025, EKF 가 /clock 60 Hz 마다 냄). Nav2 · AMCL 이 함께 돌아 PC 부하가 커지면 최대가 조금 늘 수 있음 | **통과**. 세 방법(bag infra1 · infra2, 영상 구독 탐침) 모두 중앙 0.0083 · p95 0.025 · 최대 0.025 s, 0.1 s 넘음 0(각 10,361 장). 예상보다 작음 |
| T2 map → 앞 카메라 TF 나이 | 조회 성공 %, 중앙 · p95 · 최대(s) | 통과. 100 %, p95 0.03~0.05(11-01 0.033, 11-02 설계 45 판 odom → base_link p95 0.042~0.050). 위험: p95 가 문턱 0.05 에 붙음 | **통과**. 100 %(3,454 번), 중앙 · p95 · 최대 0.025 s. 영상 받을 때 잰 map → 영상 frame 나이도 중앙 0.0083 · 최대 0.025 s |
| 기록: 영상 받은 때 − 영상 stamp | 중앙 · 최대(s) | 0.02 s 안(영상 직렬화 · 전송, 모의 시각 기준) | 0 · 0(모의 시각이 /clock 60 Hz 단위라 같은 틱 안에 도착). 실물 값은 Jetson 지연 측정에서 |
| 기록: 영상 stamp 의 odom → base_footprint 조회 성공 | % | 99 % 안팎(EKF 가 영상보다 앞서면 바로 성공, 뒤지면 다음 EKF 때까지 기다려야 함) | **11 %**(예상과 다름). 영상 stamp 가 EKF 최신보다 보통 0.008~0.025 s 앞서서, 받은 순간 그 시각 조회는 대부분 실패한다. 인지는 이때 0.1 s 안 최근 값(tf_fallback_max_s)으로 넘어가므로 T1 안이면 동작은 같음. 정확한 시각 값을 원하면 인지가 짧게(≤ 0.03 s) 기다리는 쪽 |
| 참고(판정 밖) | | | 장면 영상 받은 수 10,388/10,410(99.8 %), stamp 간격 최대 0.033 s. EKF TF 주기 0.0167 s, AMCL map → odom 0.1 s · stamp 가 받은 때보다 +0.283 s(transform_tolerance 0.3). 같은 판 Nav2 채점은 G3 불통과(겹침 4 표본, −0.001 m) · 위치 SUSPECT → LOST → 다시 찾음 1 번(t 215~218 s) · SUSPECT 1 번 — 팀 EKF vy 없음(Issue #92) · Nav2 출발 흔들림으로 아는 것, 이 시험 기준 밖 |

- 결론(모의): 카메라 켠 실행에서도 인지 조건 1(영상 stamp − EKF ≤ 0.1 s)과 TF 나이 p95 ≤ 0.05 s 를 넉넉히 지킨다. 실물(RealSense 드라이버 stamp · Jetson 부하)은 Jetson 지연 측정에서 다시 잰다.
