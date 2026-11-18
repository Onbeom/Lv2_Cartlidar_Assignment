## 요청 (인지 파트 → opencr_hardware)

`opencr_hardware/urdf/robot.urdf.xacro`, `config/controllers_4wd.yaml` 의 바퀴 값이 기준 URDF 261027_1711 과 다릅니다. 어느 쪽이 맞는지 확인 부탁드립니다(실물 측정값이 있으면 그 값이 우선).

| 항목 | opencr_hardware | 기준 URDF 261027_1711 |
|---|---|---|
| 바퀴 중심 \|y\| | 0.1585 m | 0.1546 m |
| 앞바퀴 x · 뒷바퀴 x | 0.0925 · −0.0675 m | 0.08 · −0.08 m |
| wheel_separation | 0.317 m | 0.3092 m |
| 바퀴 속도 한계 · effort | — | 10.995574 rad/s · 1.8 N·m (벨트 1 : 1.5) |
