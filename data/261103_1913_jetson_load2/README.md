# 261103_1913_jetson_load2 — Jetson 부하 측정 2단계 준비(카메라 3대 + YOLO + 위치 추정 묶음, 주행 없음)

## 무엇 · 왜
- 1단계(`261103_1544_jetson_load/`, 30분 통과: CPU 평균 26 % · GPU 53 % · RAM 2.4 GB) 위에 팀이 쓰기로 한 위치 추정 묶음을 더해 Jetson 여유를 다시 잰다.
- 묶음 = 팀 EKF(`bringup/config/ekf.yaml` 사본) + localization 패키지(PR #91 커밋 6613a9b 사본, `localization:=amcl use_amcl_node:=true`):
  ekf_filter_node · scan_body_mask · scan_merge · amcl(매 스캔 갱신, `config/amcl.yaml`) · map_server 2 · loc_monitor(잔차 interp_p75 · 문턱 0.034 · pair). Nav2 는 넣지 않는다(주행 담당에게 인계, 실물 주행 불가).
  - AMCL · Nav2 운용은 주행 담당(WindForce08)에게 넘겼다. AMCL 은 Jetson 전체 부하를 보려고 함께 띄운다(사용자 11-03). 결과는 프로세스별(proc.csv)로 나눠 우리 몫(loc_monitor · scan_body_mask · scan_merge · map_server)과 AMCL · EKF 몫을 따로 적는다.
- 입력은 **녹화 재생**: 모의 K0 amcl 판(`261103_1502_loc_pkg/results/K0_pkg_amcl`, 351 s, 목표 7 개를 다 돈 판)에서 센서 토픽만 뽑은 `bundle/bag/replay_k0_amcl`(/scan_front · /scan_rear · /odom · /imu/data · /tf_static, 37 MB). 정답 · 출력 토픽(/sim/truth · /tf · /amcl_pose 등)은 뺐다.
  `ros2 bag play --clock 100` → 모든 노드 use_sim_time. 판마다 묶음을 새로 띄워 출발 자세에서 시작한다.
- 카메라 · YOLO 쪽은 1단계 `load_test.py` 그대로(파이썬 흉내, ROS 없음).

## 묶음(`bundle/` → Jetson 으로 복사)
| 파일 | 하는 일 |
|---|---|
| `run_load2.sh [판 수=5]` | 처음이면 `ws/` 에 패키지 빌드(colcon, symlink) → load_test.py(카메라 + YOLO, 전체 시간) · proc_sample.py 를 띄우고, 판마다 launch → 출력 녹화 → bag 재생 → 정리. 환경 값: `ENGINE`(D 엔진, 기본 `~/yolo_D/deploy/*.engine` 첫 파일), `NO_CAMS=1`(ROS 만), `ROS_DOMAIN_ID`(기본 77, LOCALHOST) |
| `launch/load2.launch.py` | EKF + localization.launch.py(amcl) |
| `tools/proc_sample.py` | 1 초마다 프로세스별 CPU % · RSS(/proc 만) |
| `tools/load_test.py` · `trt_run.py` | 1단계 사본(종료 때 join 고친 판) |
| `src/localization` · `config/ekf.yaml` | 패키지 · 팀 EKF 사본 |
- PC 분석 `pc_tools/analyze_load2.py <Jetson 결과 폴더> [--ref <PC 재생 폴더>]` → `analysis2.json`. 재생 구간(passes.csv play ~ play_end)만 본다.
- 판 하나 ≈ 375 s, 5 판 ≈ 31 분(1단계 30분 소크와 맞춤).

## 판정 기준(내가 정함 — 1단계 기준에 위치 추정 몫을 더함, 재생 구간만)
| 항목 | 통과 |
|---|---|
| ① 카메라마다 YOLO 주기 | ≥ 14.5 Hz(1단계와 같음) |
| ② 결과 나이 | p99 ≤ 100 ms, 300 ms 초과 ≤ 0.5 %, 연속 초과 ≤ 2 장(같음) |
| ③ 카메라 받기 | D435 ≥ 29 · Arducam ≥ 25 fps, 끊김 ≤ 1 %(같음) |
| ④ 여유 | CPU 평균 ≤ 60 %(1단계 50 % → 위치 추정 몫 10 %p 더함, Nav2 · 드라이버 · C++ 인지 몫 40 % 남김), GPU 평균 ≤ 60 %(같음, 묶음은 CPU 만), RAM 최대 ≤ 5.5 GB |
| ⑤ 30분 안정 | 끝 5분 나이 p99 ≤ 처음 5분 × 1.2, RAM 증가 ≤ 100 MB, 최고 온도 ≤ 85 °C(같음) |
| ⑥ 오류 | 카메라 끊김 · 예외 0, 판마다 노드 죽음 0(launch.log 'process has died' 없음) |
| ⑦ AMCL 따라옴 | /amcl_pose ≥ 9.5 Hz(스캔 10 Hz 매 스캔 갱신), 수신 − stamp p95 ≤ 0.1 s(모의 시각) |
| ⑧ loc_monitor | 검사 중앙 ≥ 9 Hz, SUSPECT · LOST 수 ≤ PC 재생 판 |
| ⑨ 위치 | /amcl_pose 정답 차 RMSE ≤ PC 재생 판 + 0.005 m(부하로 갱신이 밀려 나빠지지 않는지) |
| ⑩ EKF | /odometry/filtered ≥ 45 Hz(설정 50) |

## 예상(실행 전)
| 항목 | 값(단위) | 예상 |
|---|---|---|
| 묶음 CPU(proc 합) | 코어 하나 = 100 % | AMCL 30~60 %(입자 500~3000 × 빔 120, 10 Hz), loc_monitor 15~30 %(파이썬 numpy 10 Hz), scan_body_mask · scan_merge 각 5~10 %, EKF 3~5 %, bag 재생 · 녹화 5~10 %. 합 60~120 %(6 코어 기준 10~20 %p) |
| ④ CPU 평균 | % | 36~46 %(1단계 26 % + 10~20 %p) |
| ④ GPU · RAM | %, GB | GPU 53 % 그대로(± 3), RAM 최대 2.8~3.2 GB(묶음 + ROS 약 0.4~0.8 GB) |
| ①② YOLO | Hz, ms | 15 Hz 유지, 나이 p99 55~75 ms(CPU 경쟁으로 1단계 52~61 보다 조금 늘 수 있음) |
| ⑦ AMCL | Hz, s | 10 Hz, 수신 − stamp p95 0.02~0.05 |
| ⑧ loc_monitor | Hz | 9.5~10 |
| ⑨ 위치 RMSE | m | PC 재생과 ± 0.003 안 |
| 위험 | | AMCL 입자 3000 까지 늘 때(회전 · 정렬 구간) 순간 CPU, 파이썬 loc_monitor 가 YOLO 추론 스레드와 GIL 이 아닌 코어를 다툼 |

## Jetson 준비 확인(사용자 단계)
- 필요한 ROS 패키지: nav2_amcl · nav2_map_server · nav2_lifecycle_manager · robot_localization · rosbag2_storage_mcap · tf2_ros, 파이썬 numpy · yaml · cv2, colcon.

## PC 도구 확인(19:14~19:28, 모의 입력 재생 · 카메라 없음 · 판정 아님, `pc_check/`)
- 1차(19:14) **도구 실패 2 건**:
  - loc_monitor 가 시작하자마자 죽음: `ModuleNotFoundError: cartographer_ros_msgs`. AMCL 모드인데 파일 맨 위에서 Cartographer 메시지를 불러서다. 그동안 모의 판은 Cartographer 빌드를 함께 source 해서 드러나지 않았다. **Cartographer 를 설치하지 않은 Jetson · 팀 로봇에서도 그대로 죽는 패키지 결함** → 패키지(feat/localization, PR #91)에서 carto 일 때만 불러오게 고침. 같이 Ctrl-C 종료 때 ExternalShutdownException 으로 exit 1 나던 것(scan_body_mask · scan_merge · loc_monitor · map_odom_relay)도 조용히 끝나게 고침. 단위 시험 3 통과.
  - 출력 bag 이 안 열림: 배경 작업의 녹화기를 INT 로 멈춰(배경 작업은 SIGINT 무시) 닫히지 않음 → TERM 으로 바꾸고 닫힐 때까지 기다림(원래 run_live 스크립트와 같은 방식).
  - 1차 결과 폴더는 다시 만들 때 지워졌다(위 두 줄이 기록).
- 2차(19:22, 고친 묶음) 통과: 노드 죽음 0, 출력 bag 열림.

| 항목 | PC 재생 | 원래 모의 실시간 판(같은 입력) |
|---|---|---|
| /amcl_pose 주기 · 수신 − stamp p95 · 최대 | 10.0 Hz · 0.018 · 0.030 s | 10.0 Hz · 0.017 s(녹화 /clock 단위) |
| AMCL 정답 차 RMSE · p95 · 최대 | 0.0179 · 0.029 · 0.042 m | 0.0191 · 0.032 · 0.044 m |
| /scan_merged · EKF | 10.0 · 49.7 Hz | — |
| loc_monitor 검사 중앙(최저) · 잔차 최대 · SUSPECT/LOST | 10.0(9.3) Hz · 0.039 · 1/0 | (K0 amcl 판 SUSPECT 2/0) |
| PC 프로세스 CPU 평균(코어 하나 = 100) | loc_monitor 7.4 · scan_merge 5.1 · scan_body_mask 4.7 · EKF 1.9 · map_server 1.2 · lifecycle 0.8 %, 'amcl' 6.6 % 는 bag 재생 · 녹화가 섞임(경로 이름 replay_k0_amcl) → 분류 순서 고침 | |
- 읽는 법: 재생이 실시간 판을 재현함(⑨ 비교 기준 = PC 재생 RMSE 0.0179, ⑧ SUSPECT 1). SUSPECT 1 은 팀 EKF vy 없음(Issue #92) 때문의 잔차로 이미 아는 것.

## Jetson 실행 절차(사용자)
1. (PC) 묶음 복사 → Jetson `~/load_test2/`.
2. (Jetson) 필요한 패키지 확인. 없으면 설치(sudo, 사용자).
3. (Jetson) `ENGINE=<D 엔진> bash ~/load_test2/run_load2.sh 5`(약 31 분, 카메라 3대 연결 · 주행 없음 · 25W 모드). 첫 판이 도구 확인을 겸함.
4. (PC) 결과 폴더 가져오기 → `pc_tools/analyze_load2.py <폴더> --ref pc_check/results/20261103_192217`.

## 결과 — Jetson 실물(11-04 08:59~09:31, `results/jetson/20261104_085937/analysis2.json`)
- 5 판 모두 끝까지 돎(재생 구간 352.4~352.6 s × 5). 카메라 3대 + YOLO D(`yolo11n_task4_D_fp32_notf32.engine`) 전체 1906 s. 11-03 19:46 판(`results/jetson/20261103_194613`)은 사용자가 1 판 중간에 끈 것이라 판정에 안 씀.
- 도구 고침(분석만, 측정은 그대로): proc_sample 의 이름 맞춤이 명령줄 낱말이라 Jetson 경로(`~/load_test2/…`)에서 EKF · scan_body_mask · scan_merge · loc_monitor 가 'load_test' 로 잡힘 → analyze_load2.py 가 launch.log 의 pid 로 노드 이름을 다시 붙임. loc_monitor 검사 0 인 판에서 분석이 멈추던 것(res_max None)도 고침. 판 4 재생 중(09:21:3x) 명령줄에 'amcl' 이 든 다른 프로세스가 3 초(평균 88 %) 잡힘 — 묶음 밖(사용자 확인 명령으로 봄), 평균에 거의 영향 없음.

| 기준 | 결과 | 판정 |
|---|---|---|
| ① YOLO 주기 ≥ 14.5 Hz | 앞 · 뒤 · 팔 15.0 · 15.0 · 15.0 Hz | 통과 |
| ② 나이 p99 ≤ 100 ms 등 | p99 50.4 · 62.0 · 55.7 ms(최대 77.6 · 75.7 · 82.9), 300 ms 초과 0 | 통과 |
| ③ 카메라 받기 | 57,182 · 57,178 · 58,250 장 / 1906 s = 30.0 · 30.0 · 30.6 fps, 끊김 8 · 8 · 0(0.014 %) | 통과 |
| ④ CPU ≤ 60 %, GPU ≤ 60 %, RAM ≤ 5.5 GB | CPU 평균 49.8 %(p95 56.6, 최대 82.2; 판별 50.3 · 45.9 · 44.6 · 56.3 · 54.2), GPU 평균 54.9 %(p95 75), RAM 최대 1.89 GB | 통과(판 4 · 5 는 아래 loc_monitor 폭주 몫 약 9 %p 포함) |
| ⑤ 30분 안정 | 나이 p99 처음 → 끝 5분 50.6 → 50.0 · 61.7 → 61.8 · 56.1 → 55.4 ms, RAM 증가 86 MB, 최고 61.9 °C | 통과 |
| ⑥ 오류 | 카메라 예외 0, 노드 죽음 0. 판 2 · 3 의 'Exception in thread' 는 종료 신호(SIGINT) 뒤 TF 리스너 스레드가 낸 것(동작 중 아님) | 통과 |
| ⑦ AMCL | 5 판 모두 10.0 Hz, 수신 − stamp p95 0.037 · 0.037 · 0.037 · 0.047 · 0.037 s | 통과 |
| ⑧ loc_monitor ≥ 9 Hz, SUSPECT · LOST ≤ PC(1 · 0) | 판 1~3: 검사 중앙 10 Hz(최저 8.2~8.4), SUSPECT 1 · 3 · 0, LOST 0. **판 4: 검사 0 번(건너뜀 3,056), CPU 98 %. 판 5: 검사 0 번, 시작 2.4 s 에 'gap'(amcl_pose 0.9 s 끊김)으로 LOST → 전역 찾기 → /initialpose 다시 넣기 → … LOST 6 번 · 끝 FAILED, CPU 102 %** | **불통과** |
| ⑨ RMSE ≤ 0.0229 m(PC 0.0179 + 0.005) | 0.0201 · 0.0196 · 0.0192 · 0.0192 · **0.0873**(최대 0.443 m) | **불통과**(판 5: loc_monitor 가 AMCL 에 틀린 /initialpose 를 넣어 생김. AMCL 자체는 판 1~4 통과) |
| ⑩ EKF ≥ 45 Hz | 49.3 · 49.5 · 50.0 · 49.5 · 49.6 Hz | 통과 |

### 프로세스별 CPU(정상인 판 1~3 평균, 코어 하나 = 100 %, 전체 6 코어의 몫 = ÷ 6)
| 묶음 | 프로세스 | CPU % | 전체 몫 %p | 예상 |
|---|---|---|---|---|
| 카메라 + YOLO | load_test.py(파이썬 흉내) | 97.4 | 16.2 | (1단계 tegrastats 26 %) |
| 우리 몫 | loc_monitor | 47.1(판 4 · 5: 98~102) | 7.9(16.7) | 15~30 |
| 우리 몫 | scan_merge · scan_body_mask | 28.8 · 26.9 | 4.8 · 4.5 | 각 5~10 |
| 우리 몫 | map_server × 2 · lifecycle_manager × 2 | 7.0 · 8.8 | 1.2 · 1.5 | — |
| 주행 담당 | amcl | 13.2 | 2.2 | 30~60 |
| 팀 | ekf_node | 11.1 | 1.9 | 3~5 |
| 시험 도구 | ros2 bag 재생 · 녹화(실물에선 라이다 드라이버) | 14.8 | 2.5 | 5~10 |
- 합 약 255 % → 42.5 %p, tegrastats CPU 판 1~3 평균 44.6~50.3 %(나머지 = 시스템 · tegrastats · ROS 데몬).
- 읽는 법: 위치 추정 묶음(EKF · AMCL 포함) 전체 ≈ 24 %p(우리 몫 ≈ 20 %p, AMCL 2.2, EKF 1.9). 예상과 반대로 AMCL(C++)은 작고 **파이썬 노드(loc_monitor · scan_merge · scan_body_mask)가 크다**. 1단계 26 % + 이번 묶음 ≈ 50 %. Nav2 · 실물 드라이버 · C++ 인지 몫으로 남은 CPU 는 약 50 %.
- map_server · lifecycle_manager 가 하는 일 없이 3.5~4.5 % 씩 쓰는 것은 모의 시각(/clock 100 Hz 구독)과 DDS 몫으로 봄 — 실물(/clock 없음)에선 줄 것(실물에서 확인할 것).

### 판 4 · 5 loc_monitor 이상
- 재생을 시작하고 3 초 안에 CPU 가 47 % → 98~150 % 로 오르고 판 끝까지 그대로. 재생 전(빈 상태)은 2 % 로 판 1~3 과 같음. 같은 판의 AMCL · EKF · scan_merge 는 정상(AMCL 10 Hz, 판 4 RMSE 0.0192).
- 검사가 모두 '건너뜀'(check() 의 TF 조회 — map → base_link 두 번 · odom → base_link, 각 0.1 s 기다림 — 이 실패). 판 5 는 같은 상태에서 amcl_pose 받는 콜백도 늦어져 'gap' LOST 가 나고, 다시 찾기가 맞지 않는 자세를 /initialpose 로 넣어 AMCL 을 망가뜨림 → 실물에서 일어나면 **감시 노드가 멀쩡한 AMCL 을 틀린 자세로 되돌리는 결함**.
- 재현(`pc_repro/`, PC 를 `taskset -c 0,1` 두 코어로 묶고 NO_CAMS=1 3 판): **3 판 모두 같은 상태** — loc_monitor CPU 94.7 · 83.4 · 94.2 %, 검사 51 · 531 · 34 번(건너뜀 3,078 · 2,603 · 3,055). AMCL 은 정상(RMSE 0.0197 · 0.0178 · 0.0199). 제한 없는 PC(24 코어, `pc_check/`)에서는 안 생김.
- 원인(`pc_repro/diag/`, 스레드별 CPU + faulthandler 스택 3 번): 주 스레드(MultiThreadedExecutor 의 대기 집합 만들기 반복) 95 %, 작업 스레드 4 개는 **모두 check() · on_tracked() 안 `lookup_transform(timeout 0.1 s)` 에서 TF 를 기다리며 잠김**(tf2 파이썬 Buffer 는 0.02 s 마다 깨어 다시 봄).
  - 순서: 스캔 시각의 TF(EKF 가 스캔보다 조금 늦음)를 콜백 안에서 기다림 → 짝 검사 20 회/s × 조회 3 번 × 최대 0.1 s 가 작업 스레드 4 개를 다 막음 → 실행기 주 스레드가 할 일이 쌓인 채 헛돌며 GIL 을 차지 → TF 리스너 스레드가 /tf 를 늦게 받음 → 조회가 더 실패 → 스스로 유지되는 막힘. 코어가 많으면 리스너가 따라와 빠져나오고, 적으면(Jetson 6 코어에 카메라 · YOLO 가 함께) 한 번 들어가면 판 끝까지 못 나옴(판 1~3 은 안 들어감, 판 4 · 5 는 재생 시작 때 들어감).
- 고침(팀 저장소 feat/localization `loc_monitor.py`, PR #91): 콜백 안 TF 조회는 기다리지 않음(timeout 0). 스캔 시각 TF 가 아직이면 짝을 줄에 두고 다음 스캔 때 다시 검사, `tf_wait_s`(0.3 s) 넘으면 건너뜀. 실행기는 SingleThreadedExecutor(다시 찾기는 원래대로 따로 작업 스레드, 그 안의 조회만 0.1 s 기다림).
- 고친 판 확인(`pc_fix/`, 같은 재생 입력, NO_CAMS=1, 도구 확인 · 판정 아님):

| 조건 | loc_monitor CPU % | 검사 수(건너뜀) | 검사 Hz 중앙(최저) | AMCL RMSE m | SUSPECT · LOST(시각 s) |
|---|---|---|---|---|---|
| 옛 판 · 2 코어 3 판(`pc_repro/`) | 94.7 · 83.4 · 94.2 | 51 · 531 · 34(3,078 · 2,603 · 3,055) | — | 0.0197 · 0.0178 · 0.0199 | 0 · 0 |
| **고친 판 · 2 코어 3 판** | **5.3 · 6.6 · 9.2** | **3,467 · 3,464 · 3,470**(29 · 27 · 26) | 10(6.4~7.0) | 0.0188 · 0.0194 · 0.0192 | 1 · 2 · 2(293 · 226+293 · 238+293), LOST 0 |
| 옛 판 · 제한 없음(`pc_check/`, ⑧ ⑨ 기준 판) | 7.4 | 3,493(3) | 10(9.3) | 0.0179 | 1(293), LOST 0 |
| 고친 판 · 제한 없음 1 판 | 10.5(PC 에 다른 세션 component_container 24 % 함께 돎) | 3,480(13) | 10(8.0) | 0.0195 | 2(238 · 293), LOST 0 |
| (참고) 옛 판 Jetson 판 1~3 | 47 | 3,482 · 3,484 · 3,484 | 10(8.2~8.4) | 0.0201 · 0.0196 · 0.0192 | 1 · 3 · 0(226 · 226+247+293 · 없음) |
- 읽는 법: 막힘이 없어짐(2 코어에서도 검사 10 Hz, CPU 1/10). 감지는 같음 — SUSPECT 는 옛 판 · 고친 판 모두 같은 자리(226 · 238 · 247 · 293 s, 각 0.1~0.7 s, 팀 EKF vy 없음 Issue #92 로 아는 잔차)에서만 나고 판마다 0~3 번 오르내림(AMCL 입자 무작위). LOST 0.
- 같이 본 것: 고친 판 2 코어 판 2 · 3 에서 scan_body_mask · scan_merge 가 **종료 신호 뒤** `RCLError: context is not valid`(대기 집합 만드는 중 context 닫힘)로 exit 1 → launch 가 'process has died' 로 적음. Jetson 판 2 · 3 의 같은 예외(TF 리스너 스레드)와 같은 종료 경쟁. 동작 중 죽음은 아님. 고침: 네 노드 main 에서 context 가 이미 닫힌 때의 RCLError 만 조용히 끝냄(따로 커밋). 정상 SIGINT 종료 코드 0 확인(노드마다 8 번), 경쟁 자체는 재현 못 함.

### Jetson 다시 재기(고친 판) — 기준 바꿈(실행 전)
- ⑧ 을 'SUSPECT 수 ≤ PC 재생 판(1)' → **'검사 중앙 ≥ 9 Hz, LOST 0, SUSPECT ≤ 3 / 판'** 으로 바꿈. 이유: 같은 입력 · 같은 코드로도 SUSPECT 가 판마다 0~3(위 표). 1 판짜리 PC 기준으로는 판정이 무작위가 됨.
- ⑥ 노드 죽음은 재생 구간 안(종료 신호 전)만 셈. 나머지 기준은 그대로.
- 묶음 갱신(11-04): `bundle/src/localization` = 팀 feat/localization(ffeb997 + 종료 경쟁 고침), `bundle/tools/proc_sample.py` 이름 맞춤 순서 고침(노드 이름 먼저, 'nav2_amcl' · 'load_test.py' 로 좁힘).
- 절차(사용자): (PC) `scp -r bundle/src bundle/tools` → Jetson `~/load_test2/` 덮어씀 → (Jetson) 남은 프로세스 정리 → `run_load2.sh 5`(약 31 분) → (PC) 결과 가져와 `pc_tools/analyze_load2.py <폴더> --ref pc_check/results/20261103_192217`.
- 11-04 12:08 · 12:15 재측정 시도 2 번 무효(`results/jetson/20261104_120850`, `20261104_121559`): `ENGINE=$(ls ~/yolo_D/deploy/*.engine | head -1)` 이 새로 생긴 `yolo11n_task4_D_384_fp16.engine`(입력 384×640)을 골라 load_test.py 가 시작하자마자 죽음(입력 640×640 가정) → 카메라 · YOLO 없이 ROS 1 판만 돌고 끝. 고침: run_load2.sh 기본 엔진을 `yolo11n_task4_D_fp32_notf32.engine` 으로 고정, load_test 가 시작 8 s 안에 죽으면 바로 멈춤.
  - 그 1 판(카메라 없음, 고친 loc_monitor, 도구 확인): loc_monitor CPU 35.7 %(옛 판 카메라 있을 때 47), 검사 3,484(건너뜀 3), SUSPECT 3 · LOST 0, AMCL RMSE 0.0194 · 10 Hz, EKF 49.8 Hz, 노드 죽음 0.

## 결과 2 — Jetson 실물, 고친 loc_monitor(11-04 13:05~13:37, `results/jetson/20261104_130515/analysis2.json`) — **판정에 쓰는 판**
- 묶음 = feat/localization 00145b9(ffeb997 TF 기다림 없앰 + 종료 경쟁), 엔진 `yolo11n_task4_D_fp32_notf32.engine`(앞 판과 같음), 카메라 3대, 5 판(재생 구간 352.2~353.0 s × 5), 전체 1906 s. 기준은 위 'Jetson 다시 재기'(⑧ 바꾼 것)대로.

| 기준 | 결과 | 판정 |
|---|---|---|
| ① YOLO 주기 ≥ 14.5 Hz | 15.0 · 15.0 · 15.0 Hz | 통과 |
| ② 나이 p99 ≤ 100 ms 등 | p99 50.8 · 58.3 · 55.6 ms(최대 69.4 · 113.5 · 72.8), 300 ms 초과 0 | 통과 |
| ③ 카메라 받기 | 57,186 · 57,183 · 58,251 장 / 1906 s = 30.0 · 30.0 · 30.6 fps, 끊김 4 · 3 · 0 | 통과 |
| ④ CPU ≤ 60 %, GPU ≤ 60 %, RAM ≤ 5.5 GB | CPU 평균 44.0 %(p95 48.4, 최대 51.2; 판별 43.4 · 48.2 · 42.8 · 44.7 · 43.3), GPU 평균 50.2 %(p95 70), RAM 최대 1.72 GB | 통과 |
| ⑤ 30분 안정 | 나이 p99 처음 → 끝 5분 50.8 → 51.0 · 57.6 → 58.8 · 55.7 → 56.4 ms, RAM 증가 44 MB, 최고 62.2 °C | 통과 |
| ⑥ 오류 | 카메라 예외 0, 노드 죽음 0(종료 때 포함 'process has died' 0) | 통과 |
| ⑦ AMCL ≥ 9.5 Hz, 수신 − stamp p95 ≤ 0.1 s | 10.0 Hz × 5, p95 0.027~0.037 s | 통과 |
| ⑧ 검사 ≥ 9 Hz, LOST 0, SUSPECT ≤ 3 | 검사 중앙 10 Hz × 5(최저 7.3~9.3, 판마다 3,473~3,493 번, 건너뜀 3~23), SUSPECT 0 · 2 · 2 · 1 · 1, **LOST 0 · 1 · 0 · 0 · 0** | **불통과(판 2 LOST 1)** — 부하 때문 아님, 아래 |
| ⑨ RMSE ≤ 0.0229 m | 0.0198 · 0.0197 · 0.0200 · 0.0193 · 0.0199 | 통과 |
| ⑩ EKF ≥ 45 Hz | 49.2 · 49.9 · 50.0 · 50.0 · 49.5 Hz | 통과 |

- 판 2 LOST(t 224.6 SUSPECT → 226.1 LOST 'residual' 2.0 s → 근처 찾기 0.61 s → /initialpose (2.549, 4.568, 1.614) → 228.6 OK, 2.5 s): 잔차가 알려진 자리(226 s, 다른 판도 이 자리에서 SUSPECT)에서 문턱 0.034 를 2 s 넘김. **그때 AMCL 정답 차 0.027 m**(판 1 · 3 같은 자리 0.023 · 0.028, 그 판들은 LOST 아님) → 거짓 경보. 다시 넣은 자세는 정답과 1.6 cm, 뒤 오차 0.006~0.007 m 로 해 없음. 같은 거짓 LOST 가 PC 모의 실시간 판에도 있었음(A-2 `261103_1909_cam_timing/results/A2_amcl`, t 216.8 · 'residual' · 2.0 s) → Jetson 부하가 아니라 **잔차 문턱 · 팀 EKF vy 없음(Issue #92)** 몫. 할 일: C8(실물 기록으로 문턱 다시), B4(#92 반영 뒤 다시).
- 부하로 본 loc_monitor: 5 판 모두 검사 10 Hz 를 따라감, 막힘 없음(앞 판 4 · 5 의 검사 0 → 이번 3,473~3,493).

### 프로세스별 CPU(5 판 평균, 코어 하나 = 100 %, 전체 몫 = ÷ 6)
| 묶음 | 프로세스 | CPU % | 전체 몫 %p | 앞 판(옛 loc_monitor, 판 1~3) |
|---|---|---|---|---|
| 카메라 + YOLO | load_test.py(파이썬 흉내) | 99.5 | 16.6 | 97.4 |
| 우리 몫 | loc_monitor | **33.4** | 5.6 | 47.1 |
| 우리 몫 | scan_merge · scan_body_mask | 29.4 · 26.9 | 4.9 · 4.5 | 28.8 · 26.9 |
| 우리 몫 | map_server × 2 · lifecycle_manager × 2 | 6.8 · 8.7 | 1.1 · 1.5 | 7.0 · 8.8 |
| 주행 담당 | amcl | 13.1 | 2.2 | 13.2 |
| 팀 | ekf_node | 10.8 | 1.8 | 11.1 |
| 시험 도구 | ros2 bag 재생 · 녹화 | 14.6 | 2.4 | 14.8 |
- 합 243 % → 40.5 %p, tegrastats CPU 평균 44.0 %. 우리 몫 ≈ 17.5 %p, 위치 추정 묶음 전체(AMCL · EKF 포함) ≈ 21.5 %p. loc_monitor 는 실행기 하나로 바꿔 47 → 33 %(−29 %).
- 남은 CPU ≈ 56 %(Nav2 · 실물 카메라 드라이버(ROS) · 라이다 드라이버 · C++ 인지 · 팔 · OpenCR 다리 몫, 이번에 안 잼). 재생 · 녹화 2.4 %p 는 실물에서 라이다 드라이버로 바뀜. /clock(모의 시각) 몫은 실물에서 빠짐.
- 예상과 비교: CPU 36~46 % 예상 → 44.0 %(안). AMCL 30~60 % 예상 → 13 %(작음), loc_monitor 15~30 → 33(조금 큼), scan 노드 각 5~10 → 27~29(큼, 파이썬 · 10 Hz 스캔 두 개). YOLO 나이 p99 55~75 예상 → 51~58(작음). RAM 2.8~3.2 GB 예상 → 1.72 GB(1단계 2.40 GB 보다도 작음 — 1단계 때 보드에 다른 것이 떠 있었는지는 모름).
- 한계: 카메라는 파이썬 흉내(RealSense SDK · V4L2 직접, ROS 카메라 드라이버 아님), 나이는 받은 뒤부터(노출 · USB 전송 제외). 주행 · 실물 라이다 없음(녹화 재생).
