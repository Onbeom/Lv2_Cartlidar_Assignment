# Isaac Sim 6.1 로봇 프로젝트 준비 이 한 일 — 보고서 재료 (작성: 2026-11-17)

규칙대로 숫자는 결과 파일에 있는 것만 썼다. 이 세션의 실험은 모두 **모의**(Gazebo)이고 실물 시험은 없다.

## 1. 맡은 일 · 기간
- 기간: 2026-10-13 ~ 2026-10-19. 모듈을 시작할 때 환경을 준비했고, 운영 측 예제(`waffle-lyrical-slam`) README 순서대로 빌드하고 지도를 만들었다. Cartographer 설정 비교도 이 세션이 했다.
- 예제에서는 TurtleBot3 Waffle Pi SDF와 미로 데이터만 가져다 썼다(허락 받음). 예제 코드는 반영하지 않았다.

## 2. 한 일
### 2-1. PC 환경 준비 (PC, 실물 장비 아님)
- 왜: Isaac Sim 6.1, ROS 2 Lyrical, Isaac ROS 5.0(cuVSLAM)이 한 PC에서 함께 도는지 먼저 확인해야 했다.
- 어떻게: `nvidia-smi`로 확인했다. 드라이버 595.99.02, CUDA 13.2, RTX 5060 Laptop 8151 MiB였다. Isaac Sim 호환성 검사를 돌렸고, Isaac ROS는 Docker 없이 apt로 설치했다(CUDA·Jetson 저장소와 NVIDIA rosdep 목록 추가). Isaac Sim은 pip 6.1.0.0으로 `~/isaacsim-venv`에 설치했다. ROS 도메인은 26으로 정했다(`~/.bashrc`).
- 결과: 호환성 검사는 **FAILED**였다. 원인은 VRAM 하나다(8.55 GB, 기준 10 GB). `ros-lyrical-isaac-ros-cuvslam` 5.0.0 외 244개 패키지는 설치를 마쳤다.
- 결론: 메모리 기준에 못 미쳐도 진행하기로 했다(사용자 결정, 실행은 됨). Lyrical 브리지 검증은 보류했다.

### 2-2. README 빌드와 Gazebo 미로 지도 (모의)
- 왜: Cartographer 비교와 Nav2에 쓸 지도와 bag이 필요했다.
- 어떻게: Cartographer를 소스로 빌드했다. 미로 월드와 Waffle Pi SDF를 띄우는 launch를 새로 썼다(`GZ_PARTITION=pa4_26`, 시작 위치 (-1.835, 2.757)). 키보드로 몰아 지도를 저장했다.
- 결과: `maps/waffle_pi_maze.pgm/.yaml`(0.05 m). 비교용 bag `bag_20261014_1724`는 1308 s, `/scan` 12,870개다.

### 2-3. Cartographer 설정 8가지 비교 (모의, 오프라인 처리)
- 왜: 기본 설정(c1)의 지도에 유령 벽이 많았다.
- 어떻게: 같은 bag을 `cartographer_offline_node`로 처리했다. 변수는 3개를 A/B로 두었다: 해상도 0.05/0.025 m, ceres 가중치 10·40/1·4, motion_filter 각도 0.1°/1°. 정답은 `maze.stl`을 높이 0.131 m에서 자른 단면이고, 정렬 범위는 ±0.30 m·±5°다.
- 판정 기준: 유령 벽 비율이 기준 이하, 벽 위치 오차가 기준 미만, 채움률이 기준보다 0.05 넘게 낮지 않을 것.
- 결과: c1은 유령 0.269, 오차 8.07 cm, 채움률 0.428, CPU 290.1 s였다. **c6(0.025 m, 10·40, 1°)은 0.035, 2.28 cm, 0.546, 384.0 s**였다. c3(가중치만 1·4)은 0.452, 15.00 cm, 0.289로 채움률에서 탈락했다. 변수별 평균 효과는 다음과 같다. 각도: 유령 −0.192, 오차 −5.18 cm, CPU −82.6 s. 해상도: −0.118, −3.91 cm, +162.6 s. 가중치: +0.064, +2.25 cm.
- 결론: c6을 채택했다. 회전 각도를 1°로 바꾼 것이 효과가 가장 컸고, 가중치를 1/4로 낮추면 나빠졌다.

### 2-4. 루프 폐합 주기 90 vs 30 (모의, 실시간 재생)
- 왜: 오프라인 처리는 끝에 전체 최적화를 하므로 주기의 효과가 지워진다. 그래서 도중 지도로 비교했다.
- 어떻게: c6 설정에서 `optimize_every_n_nodes`만 90과 30으로 바꿨다. bag을 1배속으로 재생했고, 30초마다 `/map`을 저장했다(실행마다 42장). 설정마다 3번 실행했다. 설정이 실제로 들어갔는지는 로그 `Remaining work items` 줄 수로 확인했다(90은 47번, 30은 134번).
- 결과(중앙값): 90은 도중 유령 0.053, 오차 2.84 cm, 마지막 유령 0.023, CPU 277.8 s였다. 30은 0.056, 2.82 cm, 0.028, 292.3 s였다.
- 결론: 차이가 없어서 **90(기본값)을 유지**했다(사용자 확정).

## 3. 실패 · 다시 한 것
- 처음 정답으로 쓴 SDF 충돌 박스에는 세로 판넬이 없었다(가로 40개, 기둥 83개뿐). 그래서 제대로 그려진 세로 벽이 유령 벽으로 세어졌다. 정답을 STL 단면으로 바꿨고, 처음 결과는 `summary_sdf_collision_gt.md`에 남겼다.
- PGM이 회색조라서(모르는 칸 = 128) 벽 칸이 0으로 나왔다. map_server와 같은 확률 기준(점유 > 0.65, 빈칸 < 0.196)으로 읽게 고쳤다.
- 정렬 범위 ±0.10 m·±2°에서는 값이 범위 끝에 걸렸다. 그래서 ±0.30 m·±5°로 넓혔다.
- 같은 설정을 3번 돌린 결과가 md5까지 같았다(오프라인 처리는 결정적이다). 사용자 동의를 받아 c3~c8은 1번씩만 돌렸다.
- bag의 바퀴 오도메트리 경로는 미로 밖으로 크게 벗어났다. Gazebo의 실제 로봇 위치는 녹화하지 않아서 원인은 확정하지 못했다.

## 4. 결과물 위치
- launch: `sim/gazebo_maze_waffle_pi.launch.py`
- 실험 스크립트: `experiments/cartographer_ab/` (make_configs.py, run_offline.sh, evaluate_maps.py, run_live_period.sh, snapshot_map.py, evaluate_snapshots.py)
- 결과: `results/cartographer_ab/offline_20261014/` (summary.md, notes.md, configs/), `results/cartographer_ab/live_period_20261015/` (summary.md, notes.md, snapshot_metrics.csv)
- 지도: `maps/waffle_pi_maze.*`
- TIL: `~/git/Pa_TIL/PA_1_TIL/261019/ROS2.md`(커밋 9fd3265). 261014 TIL은 TIL 세션이 커밋했다(ded4f1e).
- 팀 저장소 브랜치, PR, Issue: 없음.

## 5. 보고서에 꼭 넣으라고 사용자가 정한 것
- 팀 코드를 가져오면 무엇을, 왜, 어디에 썼는지, 무엇을 고쳤는지 적는다(사용자 2026-10-13). **이 세션에서는 팀 코드를 가져오지 않았다.**
- 예제 저장소에서 쓴 것은 Waffle Pi SDF와 미로 데이터뿐이다(허락 받음).
- 결정의 이유와 조건이 바뀐 것은 짧은 표로 넣는다(3절의 정답 교체, 반복 횟수, 정렬 범위).

## 6. 영상 · 사진
- `results/cartographer_ab/offline_20261014/overlay_stl_c1_c3_c6_c8.png`: c1·c3·c6·c8 지도를 STL 정답과 겹친 그림
- `results/cartographer_ab/offline_20261014/overlay_c1_c3_c6_c8.png`: 같은 그림의 SDF 충돌 박스 정답 판(옛 판)
- `results/cartographer_ab/live_period_20261015/period_timeseries.png`: 주기 90과 30에서 도중 유령 벽 비율과 오차가 시간에 따라 바뀌는 모습
- `maps/waffle_pi_maze.pgm`: README대로 처음 만든 지도
- 영상: 없음

## 7. 한계 · 남은 일
- 모두 Waffle Pi SDF와 Gazebo로 한 모의 시험이다. 실제 4륜 차체나 Isaac Sim에서는 하지 않았다.
- bag 하나로만 비교했다. 각 설정은 1번씩 돌렸다(c1·c2만 3번).
- 이 세션이 끝날 때 남긴 일은 Nav2, Isaac Sim, cuVSLAM, Lyrical 브리지 검증이다. 이 일들은 뒤 세션들이 이어받았다.
