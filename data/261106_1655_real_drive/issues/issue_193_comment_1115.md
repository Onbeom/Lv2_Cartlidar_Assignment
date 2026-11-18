@kanichong 1 번 · 2 번 답입니다 (frame · timeout 은 developlcy-oss 답대로).

1. **누운 퍽도 `turn_p17` · `turn_m17` 회전을 합니다.** 세우기 정책(`lie_upright`, r2g2)은 학습 환경에서 로봇 중심 (x −0.04035) 둘레로 s × 17.06° 돈 상태에서 시작하도록 학습했습니다(`lieupright_env_cfg.py`). 회전 없이 쓰면 학습하지 않은 조건이 됩니다. 순서는 제안하신 줄 그대로 `turn` → `target_look` → `lie_upright` → `toppick_tilt` → `pallet_load_…_tilt`, 실은 뒤 `turn_0`.
2. **B6(inline, W 면)은 예외로 가까운 퍽을 먼저 집습니다** — 세운 퍽 먼저 규칙을 B6 에만 적용하지 않음. 먼 세운 퍽을 누운 퍽 너머로 먼저 집으면 팔이 누운 퍽에 닿을 수 있어서입니다.
   - 참고: B6 는 회전이 없고 누운 퍽이 정면 0.227 m 라, 세우기 정책이 학습한 조건(17° 돌고 옆 ±40 mm)과 다릅니다. B6 에서 누운 퍽을 세우는 동작은 실물 · Isaac 확인 전입니다.
