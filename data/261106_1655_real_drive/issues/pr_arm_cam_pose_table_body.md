## 무엇을
팔 카메라(Arducam) 자세별 실측 보정 (#278). 촬영 시각 팔(link5) 자세가 보드 사진을 찍은 자세 근처(0.03 m · 0.10 rad)면 link5 → 카메라 장착을 실측 값으로 바꿔 씀, 밖이면 설계 FK 그대로. `arm_cam_correction.hpp`(새) + `perception_node` `lookup_at` → `cam_correct()`, 설정 `perception.yaml` `arm.cam_correction`(BOXLOOK_PLACE · BOXLOOK_TAG 표).

## 왜
팔 링크 길이(#295) · 영점(#292)이 정해지기 전이라 FK 의 팔 카메라 자리가 자세마다 다르게 틀림(보드 사진: PLACE 높이 −10 mm, TAG −23 mm). URDF 는 그대로 두고 인지 쪽에서 쓰는 자세만 맞춤.

## 시험
- PC: `test_arm_cam_correction` 3 개(표 읽기 · 가까운 항목 고르기 · 보정 합성) 통과, perception ctest 8/8, 노드 시작 줄 확인.
- 표의 link5 자세 = 펌웨어 관절각 + 이 URDF FK = 실행 중 robot_state_publisher 와 같은 기준(다시 낸 차 0.05 mm · 0.006°). 보정 뒤 잔차(맞춤에 쓴 사진): 높이 ≤ 1.2 mm, 방향 PLACE 0.22° · TAG 0.54°(기준 0.5° 넘음).

## 한계
- **enabled false 로 들어감.** 켜는 조건은 yaml 주석(실물에서 보정 줄이 뜨는지 · 맞춤에 안 쓴 보드 사진의 보정 뒤 높이 차 ≤ 5 mm · 미션 box_look 이 표에 드는지).
- 보정 값(x +50 mm · z +12~22 mm 등)은 실제 장착 치수가 아니라 링크 · 영점 · 보드 위치 오차를 함께 흡수한 값 → 표 경계를 넘나들 때 카메라 자리가 수 cm 뛸 수 있음. target_look 표는 아직 없음(#287 로그로 추가 예정).
