# 5번 (#154) 검토 계산 — 2026-11-10 (미션 파트 261110 1)

- `occl.py <urdf> j1 j2 j3 j4 j5`: 앞 D435 컬러 광학 원점 → 퍽(상자 중심 y ±0.04, 윗면 · 중간 높이) 선분이 팔 충돌 상자를 지나는지. 면 거리 = 차체 앞 끝 → 박스 면.
- URDF: `git -C ~/git/Lv2_Cartlidar_Assignment show origin/dev:workspace/urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf > waffle_4wd.urdf` (dev 055f705)
- 결과(계산, 모의 · 실물 아님): READY 는 면 0.45 ~ 0.127 전 구간 −Y 퍽을 link4 · link3 이 가림. ALIGN_A `[0,-1.854,0.540,1.645,0]` 는 면 0.45 ~ 0.31 가림 없음, 0.30 이하 −Y 퍽을 gripper_roll_link 가 가림.
