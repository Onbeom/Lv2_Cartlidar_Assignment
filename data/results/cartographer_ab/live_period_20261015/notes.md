# 루프 폐합 주기 비교 — 실행 조건 (2026-10-15)

모의 시험이다(Gazebo bag 재생). 실물 시험이 아니다.

- 입력: `../bag_20261014_1724` (어제 오프라인 비교와 같은 bag)
- 기준 설정: `../offline_20261014/configs/c6_res0.025_w10-40_ang1.lua` (어제 비교에서 가장 좋았던 설정)
- 변수: `POSE_GRAPH.optimize_every_n_nodes`. A = 90 (Cartographer 기본값), B = 30
- 방법: bag을 실제 시간(1배)으로 재생해 `cartographer_node`와 `cartographer_occupancy_grid_node`(해상도 0.025 m, 1초마다 발행)를 돌리고, `/map`을 시뮬레이션 시간 30초마다 저장했다(실행마다 42장). 오프라인 처리와 달리 끝에 전체 최적화를 하지 않으므로 주기의 효과가 도중 지도에 남는다. 설정마다 3번 실행했다.
- 평가: 도중 지도마다 정답(maze.stl 높이 0.131 m 단면)과 비교해 유령 벽 비율과 벽 위치 오차를 구하고, 실행마다 시간 평균을 낸 뒤 3번의 중앙값으로 비교했다. 도중 지도마다 따로 정렬(±0.30 m·±5°)한다. 초반의 작은 지도도 같은 무게로 평균에 들어간다.
- 설정이 실제로 들어갔는지: Cartographer 로그의 `Remaining work items in queue` 줄이 주기 90에서 47번, 주기 30에서 134번이었다(약 2.9배).
- 결과: `summary.md`, 시간에 따른 변화: `period_timeseries.png`, 도중 지도별 수치: `snapshot_metrics.csv`
- 처음 동작 확인(bag 앞 70초, 주기 90만)은 임시 폴더에서 했고 판정에 쓰지 않았다.
