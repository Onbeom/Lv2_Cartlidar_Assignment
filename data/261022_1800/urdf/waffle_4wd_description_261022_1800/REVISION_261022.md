# 261022 리비전 변경 기록 (2026-10-22)

원본: `../waffle_4wd_description_261021/` (바꾸지 않음)

## 바꾼 것
- `meshes/main/arm_link_5.dae`(표시 메시)에서 집게 체결 나사 세트 4개를 지웠다(나사 4 + 너트 4 = 8조각, 삼각형 4724개).
  - 이 나사들은 집게 손가락 부품인데 CAD 내보내기에서 arm_link_5 메시에 합쳐져 있었다. 실물 arm_link_5에는 없다.
  - 그대로 두면 집게가 움직여도 나사가 손목에 남아 떠 보인다.
  - 삼각형 123499 → 118775, 점 63696 → 61116. 남은 형상의 좌표는 바꾸지 않았다.
  - 지운 위치(로봇 기준, base_footprint): x 356~367 mm, y ±22~28 mm, z 299~352 mm.
- 만든 도구: `~/git/lv2_next_module/sim/tools/remove_arm_link_5_fasteners.py`

## 바꾸지 않은 것
- URDF·xacro의 관절·형상 문구, 충돌 메시, 질량·관성 값, 관절.
- 나사 질량(수 g)은 arm_link_5 질량 추정값에 그대로 남아 있다(무시할 수준).

## 추가 변경 (2026-10-22, 폴더 이동 후)
- 위치: `~/git/lv2_next_module/261022_1243/urdf/waffle_4wd_description_261022/`(원래 `mine/urdf/`에서 옮김)
- 패키지 이름을 `waffle_4wd_description_261021` → `waffle_4wd_description_261022`로 바꿨다: package.xml, CMakeLists.txt, URDF·xacro(`package://` 504곳), launch, README, robot_data.yaml. 이제 261021 패키지와 한 작업 공간에 함께 둘 수 있다.
- README 제목·안내와 "visual bounds" 문장에 261022 변경(arm_link_5 앞 끝 366.6 → 361.1 mm)을 반영했다. `simulation_metadata.json`에 `revision` 항목을 추가했다(기존 값은 그대로).
- USD(`261022_1243/usd/waffle_4wd_261022/`)는 메시를 USD 안에 담고 있어 패키지 이름과 무관하다. 다시 만들지 않았다.
