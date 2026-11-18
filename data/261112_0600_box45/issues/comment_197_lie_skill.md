누운 퍽 스킬 이름을 알려 드립니다(#193 연결용).

- **세우기**: manipulator `execute_skill` 이름 `lie_upright:<frame>:<상자 x>:<상자 y>:<상자 요 deg>` (`manipulator_node.cpp` `parseTaskSkill`). 누운 직육면체를 그 상자 위에서 세웁니다.
- **다시 집기**: 세운 뒤에는 같은 박스에서 미션의 `toppick_tilt` 고정 동작으로 집습니다(별도 스킬 이름 없음).
- **문장 순서(꼭 지킬 것)**: `target_look`(그 누운 퍽을 겨눔, 퍽 자리를 잡아 둠) → `lie_upright`(상자) → `toppick_tilt`(같은 박스). `lie_upright` 는 바로 앞 `target_look` 이 잡아 둔 누운 퍽을 씁니다(`manipulator_node.cpp` lie_upright 머리말).
- 런 나누기 제약: 같은 박스에서 세운 퍽을 먼저 집습니다. hybrid 런에 넣어 주시면 됩니다. 우리 쪽 `fix/lie-pick-order`(per_location 용, push 안 함)는 쓰지 않겠습니다.
