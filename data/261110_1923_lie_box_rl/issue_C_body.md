#192 (누운 직육면체 세우고 45° 로 다시 잡기) 를 미션에 연결하는 일도 제가 하겠습니다. 다른 담당 파트에는 PR 리뷰만 부탁드립니다.

## 할 것
1. **서 있음 · 누움 판단**: 측량 (`perception_survey`) 의 pose (standing / lying) 와 집기 직전 인식 (`/perception/pucks` 의 `standing`) 으로 판단합니다.
2. **등록표 · 순서**: mission_node 가 쓰는 등록표에 pose 를 넣고, 같은 상자에 누운 퍽이 있으면 세운 퍽을 먼저 집습니다 (발제: 집기 순서 자유). mission_node 쪽에서 할 수 있는지 먼저 보고, 계획기 (`mission_lines.py`) 를 고쳐야 하면 PR 로 리뷰를 부탁드리겠습니다.
3. **스킬**: 누운 퍽이면 `toppick_tilt` 대신 누운 퍽 스킬을 부릅니다. 끝 상태는 세운 퍽 45° 집기 끝과 같아 (잡는 점 = 퍽 중심 + 15 mm, 3 cm 들림, 1 s) 그 뒤 `pallet_load_*` 는 그대로입니다. `manipulator_node` 에 스킬을 더해야 하면 `fix/` 브랜치 PR 로 올리겠습니다.

## 부탁
- @kanichong `manipulator_node` · 계획기 PR 이 올라가면 리뷰 부탁드립니다.
- @developlcy-oss 스킬 이름 규칙이나 정책 불러오는 자리에 정해 둔 것이 있으면 알려 주세요.
