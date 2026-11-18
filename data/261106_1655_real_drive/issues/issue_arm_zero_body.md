## 묻는 것
팔 관절 영점 보정(#266 의 2 번)을 했는지 알려 주세요. 했다면 언제 · 어느 값으로 바꿨는지, 안 했다면 아래 순서로 해 주시고 끝나면 이 이슈에 남겨 주세요. 인지 쪽 팔 카메라 손-눈 보정(#278)은 영점이 맞은 뒤에 다시 합니다.
bfc19e4 에서 `apply_joint_offsets` 를 끈 이유도 알려 주세요(영점 상태를 읽는 데 필요).
결과는 이 이슈에 남기고 #266 2 번 칸도 체크해 주세요(같은 일).

## 지금 보이는 것 (dev 959d7a3 기준)
- `control/config/manipulator.yaml`: `ik.joint_offsets` = [−0.028, −0.479, 0.164, 0.096, 0] (11-12 v3 칸 자세로 맞춘 처짐 포함 실효값), bfc19e4(11-14 00:39)에서 `apply_joint_offsets: false` 로 꺼짐.
- #266 의 '2. 팔 영점 보정 (v5 칸 자세로 다시)' 체크 칸이 아직 비어 있음.
- 팔 카메라로 보드를 찍어 낸 카메라 높이가 설계 FK 와 자세마다 다름: BOXLOOK_PLACE +8 ~ +10 mm, BOXLOOK_TAG +20 ~ +22 mm(11-14 새벽, #278). 관절 2 · 3 · 4 영점만 고치는 모델로는 사진 21 장을 판정 기준(높이 ≤ 3 mm · 방향 ≤ 0.5°) 안에 못 맞춤 → 카메라 장착(인지 쪽)과 영점이 섞였을 수 있어, 영점을 먼저 확정해야 카메라 쪽을 가를 수 있음.

## 안 했다면 (#266 2 번에 적힌 순서)
1. `tools/arm_bench/real/meas_session.py` 메뉴 10(P1 ~ P10 손끝) · 4(v5 10 칸) · 12(상자 위 가운데 · 좌 · 우) → `~/실측/6_보정자세.csv`
2. `python3 tools/arm_calib/arm_calib_fit.py 6_보정자세.csv --sag 2,3` → 나온 한 줄을 `control/config/manipulator.yaml` 에 (사용법 `tools/arm_calib/README.md`)
3. 결과(RMS, 새 offsets, `apply_joint_offsets` 를 다시 켤지)를 여기 남겨 주세요.
