# 02. AMCL 설정 (`amcl/amcl.yaml`)

## 출처 — kanichong 원본 + 모의 최적화
- 바탕: 팀 저장소 `mission_planner/config/amcl.yaml`(kanichong, b47463b). 노드는 표준 `nav2_amcl`.
- 환경에 맞춘 것 4 개: `global_frame_id` map(발제 좌표), `scan_topic` /scan → `/scan_merged`, `map_topic` /map_amcl → `/map`(SLAM 벽 0.01 m), `initial_pose` (0.69365, 0.95, π) → (0.734, 0.95, π)(모의 base_link 출발 참값).
- **최적화로 바꾼 것 5 개**(오프라인 재생 r1~r4, 근거는 `reference/design_docs/B_model_and_amcl_tuning_261102.md` 'AMCL 최적화'):

| 값 | kanichong | 이 설정 | 이유 |
|---|---|---|---|
| `robot_model_type` | DifferentialMotionModel | **OmniMotionModel** | 회전축이 base_link 뒤(B안 −0.0658 m)라 제자리 회전 때 base_link 가 옆으로 움직임. 차동 모델은 이 옆 이동을 앞 이동으로 넣어 옆 오차가 앞뒤의 2 배였음. 옴니로 RMSE 약 절반 |
| `alpha5` | (차동이라 안 씀) | 0.1 | 옴니 모델의 옆 이동 잡음 |
| `update_min_d` | 0.03 | 0.0 | 매 스캔 갱신(제자리 회전 · 느린 이동에서도 고침) |
| `update_min_a` | 0.05 | 0.0 | 같음 |
| `laser_likelihood_max_dist` | 1.0 | 2.0 | r1 에서 가장 작은 오차 |

나머지(입자 수, sigma_hit, z 값 등)는 kanichong 값 그대로. 레이저 값(sigma_hit 0.015~0.05, max_beams 240 · 480)은 오차 차이 ±4 % 로 거의 둔감했음.

## 결과(모의)
| 판 | RMSE · 최대(m) | 판정 |
|---|---|---|
| kanichong 원본, Nav2 설계 45, 1 판 | 0.0317 · 0.095 | G3(차체 여유) 불통과 |
| 이 설정, Nav2 설계 45, seed 3 판 | 0.0154~0.0162 · ≤ 0.030 | G1~G5 통과, G3 최소 0.0058~0.0096 m |
| 이 설정 + 팀 bringup EKF(ICR 보정 없음) + localization 패키지, K0 | 0.0221 · 0.051 | G1~G5 통과, loc_monitor SUSPECT 2 회(잔차 0.0375 · 0.0391 > 문턱 0.034, 1.4 · 0.6 s) |
| 같음 + ICR 보정 EKF, K0 | 0.0157 · 0.028 | 통과, SUSPECT 0 |
| 이 설정 + 팀 EKF, K1(정지 중 0.20 m · 10° 강제 이동) | 감지 0.52 s, `/initialpose` 다시 넣은 뒤 0.0156 m · 0.14°, 감지 → OK 3.02 s | 통과 |

- AMCL 은 강제 이동을 odom 으로 모르므로 감지는 점프가 아니라 잔차(0.5 s 이어짐)로 됨. Cartographer(0.19 s)보다 느림.
- 위험(측정 안 함): 매 스캔 갱신은 Jetson CPU 부하가 늘 수 있음. 무거우면 `update_min_d/a` 를 조금 올리는 것이 첫 후보(오차는 늘어남).
- kanichong 파일(`mission_planner/config/amcl.yaml`)을 직접 바꿀지, 새 파일로 둘지는 kanichong 과 정할 것(남의 파일).
