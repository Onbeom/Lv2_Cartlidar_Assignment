# 설계 12 = 설계 11 에서 바꾼 값(Isaac 실시간 2단계 불통과 뒤, 기준 그대로)

| 항목 | v11 | v12 | 이유 |
|---|---|---|---|
| `controller_server.FollowPath.closed_loop`(RotationShim) | `true` | `false` | Isaac 실시간(261101_0016_isaac_live/results/live_nav_v11): 회전 명령을 '측정 각속도 + max_angular_accel × 주기'로 올리는데, 4륜 스키드는 작은 회전 명령에서 차체가 안 돌아 측정 각속도 ≈ 0 → 명령이 0.8 × 0.05 = 0.04 rad/s 에 묶임(교착). 2D 모의는 명령을 그대로 따라서 안 드러남. 속도 다듬기는 이미 OPEN_LOOP |
