# 지도 어긋남 확인 (자료 비교, 실물 주행 · 모의 실행 없음)

navigation README 의 "slam_walls_001 이 동쪽으로 갈수록 SDF 와 8~9 cm, 북동 구석 15 cm 이상 어긋남"을 확인했다. 자료는 origin/dev 30c2d1f 고정 사본 `src/`. 실행: `extract_usd.py` → `compare.py`(결과 `compare.json`), 그림 `photo/261106_미로지도비교.png`.

| 항목 | 값의 의미 · 단위 | 예상 | 결과 |
|---|---|---|---|
| slam → maze.usda | SLAM 지도 점유 칸에서 우리 Isaac 충돌 형상까지 거리, m | 칸 0.01 + 벽 두께 정도로 작음(이 지도로 Isaac AMCL RMSE 0.016 이 나왔음) | 중앙 0.0096 · p95 0.0305 · 최대 0.0521, 5 cm 넘는 점 0 %. 1 m 칸별 맞춤 이동 대부분 2 cm 이내 → **SLAM 지도는 자기 세계와 맞음** |
| maze.usda → SDF | 세계끼리 차이, m | 10-30 기록의 3~13 cm | 중앙 0.0365 · p95 0.0964 · 최대 0.1429. 동쪽 벽 x 약 −0.08, 북쪽 벽 y 약 −0.07~−0.10, 안쪽 벽 x 3.5 쪽 −0.115 |
| slam → SDF | README 주장 재현, m | 8~15 cm | 중앙 0.0435 · p95 0.1095 · 최대 0.1405 → 수치는 재현 |
| 팀 Isaac 경기장 → SDF | scenes/dice_mission.usda 와 SDF 차이, m | 거의 같음(SDF 에서 옮긴 것) | 중앙 0.0007, 바깥 범위도 같음. p95 0.1376 은 SDF 잘라낸 높이에 없는 형상 쪽(더 보지 않음) |

## 해석
- 원인은 SLAM 오차가 아니라 **세계 모델이 둘**인 것. 우리 Isaac 은 10-30 사용자 결정대로 maze.zip 의 maze.usda, 팀 Gazebo · 팀 Isaac 은 발제 도면 SDF. 두 벽 위치 차이는 10-30 에 이미 기록됨(3~13 cm).
- dev 기본 `/map`(slam_walls_001) · `loc_map`(slam_walls) 은 maze.usda 세계 전용이다. SDF 세계에서 쓰면 안 된다.
- `map:=` 만 바꾸고 `loc_map:=` 을 안 바꾸면 AMCL 은 SDF 지도, loc_monitor 잔차는 maze.usda 지도라 거짓 SUSPECT · LOST 가 날 수 있다(코드 읽기로 본 것, 시험 안 함). gazebo_sim.launch.py 는 loc_monitor 를 끄고 있어 드러나지 않았다.
- 어느 세계가 실물에 가까운지는 모른다. 실물 SLAM(C7)으로만 정할 수 있다.
