# Waffle CAD AI 인수인계 — 2026-10-21

이 폴더는 무료 CAD 프로그램을 사용하는 새 AI 에이전트가 **설계를 계속 수정**하도록 만든 독립 묶음입니다. 기존 Fusion 파일을 열 수 있다고 가정하지 마세요. STEP을 먼저 사용하고 새 편집 내용은 FreeCAD의 FCStd 등 해당 프로그램의 원본 형식으로 저장하세요.

1. 새 세션에 `01_AI_SESSION_PROMPT.txt` 내용을 붙여 넣고 이 묶음을 제공하세요.
2. `02_DESIGN_STATE.md`, `03_VALIDATION_AND_PENDING.md`, `04_FREECAD_WORKFLOW.md`, `06_SIMULATION_CHANGES.md`를 읽으세요.
3. 주 편집 기준은 `CAD/Exchange/Assembly_Modular_261021.step`입니다. 분할 브라켓은 같은 폴더의 `Floor_Bracket_Modular_261021.step`입니다.
4. `CAD/Print`의 개별 STEP/STL은 **출력 방향으로 놓인 부품**입니다. 이 좌표를 전체 어셈블리 위치로 덮어쓰면 안 됩니다.
5. Fusion 원본은 `CAD/Fusion_Native`에 보존합니다. `CAD/Original_Archive`의 F3Z와 원래 브라켓은 과거 자료이며 최신 어셈블리로 쓰면 안 됩니다.
6. ROS 패키지는 `ROS/waffle_4wd_description_261021`입니다. `Data`의 물성·관절 정보와 `Evidence`의 검증 범위를 함께 보세요.
7. `Motion`은 기존 402프레임 시연의 이식 가능한 수치 데이터입니다. `Tools/FreeCAD_preview_motion.FCMacro`는 이 데이터와 링크별 메시로 미리보기를 구성하는 새 어댑터입니다. FreeCAD GUI에서의 실행은 아직 검증하지 않았습니다.

`CADai_FreeCAD_ROS_Motion.zip`은 무료 CAD 편집·ROS·동작·문서를 포함합니다. `CADai_Fusion_Archives.zip`은 큰 Fusion 원본과 과거 F3Z 보존용입니다. 두 ZIP을 같은 상위 폴더에 풀면 `CADai` 트리로 합쳐집니다. 폴더 자체가 이미 있다면 다시 풀 필요가 없습니다.

## 단위와 상태

- 사용자에게 제시하는 설계값: **mm, g, g·mm², rad, N·mm**. 각도 설명에는 degree 병기 가능.
- ROS URDF 내부에만 SI 단위 사용: m, kg, kg·m², rad. 질량·관성의 단위만 바꾸고 좌표 기준을 섞지 마세요.
- 물성은 CAD/합의한 재료에 근거한 **추정값**입니다. 실측값이나 실제 강도 검증 결과가 아닙니다.
- 최신 비적재 추정 질량 4,891.1940677g. 퍽 12개는 803.52g, 합계 5,694.7140677g. 새 체결용 볼트·너트·와셔와 배선 등 제외 항목이 있습니다.
- 전체 검증이 완벽히 끝났다고 말하지 마세요. 검증별 범위와 추가 데이터는 `03_VALIDATION_AND_PENDING.md`에 있습니다.

## 재생 코드와 CAD 수정의 차이

`Legacy_Fusion_Code/WaffleExport`는 adsk API를 사용하는 기존 코드 보존본입니다. FreeCAD에서 그대로 실행되지 않습니다. 다만 `Motion`의 관절각, 퍽 위치와 집게 벌림 값은 재사용할 수 있고 `Tools/portable_fk.py`는 Fusion 없이 그 값을 계산합니다. 메시 시연과 실제 접촉/파지 시뮬레이션은 다릅니다.

웹 AI 세션이 파일이나 FreeCAD 런타임을 실제 사용할 수 있는지는 그 세션에서 먼저 확인해야 합니다. 사용할 수 없으면 완료한 척하지 말고 실행 가능한 매크로와 필요한 입력을 제시하세요.

