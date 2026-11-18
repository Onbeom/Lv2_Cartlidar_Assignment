수정 요청 하나 더 드립니다(비스듬히 서기, #225 · #226 관련).

- **무엇**: `box_look` 을 호출할 때 거리 · 각을 하나 정해 줄 수 있게 해 주세요. 예: `box_look:<frame>:<x>:<y>:<z>:<거리 m>:<각 deg>`. 5 필드는 지금처럼 설정 목록을 씁니다.
- **왜**: 같은 자리에서 place_box 는 0.44 m · 50°, 태그 읽기는 0.40 m · 55° 로 따로 봐야 합니다(Isaac 렌더 판정 통과: place_box 224/224, 태그 98.9 %, 모의). 지금은 목록이 하나(`view_angles_deg [90 ~ 70]` · `distances [0.20 ~ 0.26]`)뿐이라 단계마다 고를 수 없고, 두 값도 목록에 없습니다.
- **우리가 해 본 것(사본, push 안 함)**: `manipulator_node.cpp` 를 고쳤고, control 시험 74 개 통과, mock 미션에서 7 필드 문자열이 나가는 것까지 확인했습니다.
  - `TaskSkill` 에 `std::optional<double> look_dist, look_deg` 를 더했습니다.
  - `parseTaskSkill` 이 box_look 만 7 필드를 받습니다(거리 > 0, 0 < 각 ≤ 90).
  - `startLook` 에서 `auto lp = look_params_.at(t.look);` 뒤에 값이 있으면 `lp.distances = {dist}; lp.view_angles = {deg · π/180};` 로 덮어씁니다. `min_clearance` 와 카메라 장착값은 그대로입니다.
  - 필요하시면 diff 를 드리겠습니다.
- 실물 팔에서는 확인하지 않았습니다.
