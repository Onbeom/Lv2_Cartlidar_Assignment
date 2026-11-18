## 요청 (인지 파트 → control)

저장 지도로 Nav2(AMCL · MPPI)를 2D 모의에서 확인하면서 control 설정과 다른 값을 찾았습니다(Isaac · 실물 시험 아님). 어느 값이 맞는지 확인 부탁드립니다.

1. `control/config/drive.yaml` `navigate.xy_goal_tolerance: 0.25  # 이 안의 목표는 nav2 spin 으로 (nav2 와 같게)`
   - 모의에서 확인한 Nav2 설정의 위치 허용 오차(goal checker `xy_goal_tolerance`)는 **0.06 m** 입니다. 태그 앞 칸은 제자리 회전 여유가 0.07 m 남짓이라 도착 오차를 크게 둘 수 없습니다.
   - 지금 값이면 현재 위치에서 0.25 m 안의 목표는 이동 없이 spin 만 하므로, 0.06~0.25 m 짧은 이동 요청이 무시됩니다. 0.06 으로 맞추거나 주석을 고쳐 주세요.
2. base_link → 앞 범퍼 거리가 세 곳에서 다릅니다.

| 위치 | 값 |
|---|---|
| `drive.yaml` `base.front_offset` | 0.20 m |
| `robot_geometry.yaml` `base.front_offset` / `rear_offset` | 0.25 / 0.25 m |
| Nav2 footprint(기준 URDF 261028 외곽) | 앞 0.263 m · 뒤 0.155 m · 옆 ±0.171 m |

   `align` 의 정지 거리(`scan.front_margin`, `coarse_min`)가 범퍼 기준이라 이 값이 틀리면 정지 거리도 같은 만큼 틀립니다. 실측값이 있으면 그 값이 우선입니다.
