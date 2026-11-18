# 무료 CAD에서 이어가기

## 가져오기와 편집

1. AI 실행 환경에서 FreeCAD 모듈/실행파일과 버전을 먼저 확인한다. 웹 채팅이 사용자 PC 파일을 자동으로 볼 수 있다고 가정하지 않는다.
2. `CAD/Exchange/Assembly_Modular_261021.step`을 FreeCAD에서 열고 하위 부품/인스턴스/배치를 확인한다. 또는 `Tools/FreeCAD_import_editable.FCMacro`를 실행한다. 이 매크로는 STEP을 가져오고 `Working_FreeCAD/Waffle_Imported_261021.FCStd`와 객체 목록을 저장한다. 기존 FCStd는 덮어쓰지 않는다. 이 GUI 매크로는 제공만 했고 이 세션에서 FreeCAD 실행 검증하지 않았다.
3. STEP 가져오기는 분석 가능한 BRep를 제공하지만 Fusion의 파라미터 히스토리와 관절을 자동 복원하는 작업이 아니다. 수정 대상은 BaseFeature/Part/Body로 관리하고 필요한 스케치·치수·구속을 재구성한다. 저장은 새 FCStd와 STEP, 출력용 STL을 별도로 한다.
4. 분할 브라켓 조립 위치는 `CAD/Exchange/Floor_Bracket_Modular_261021.step`을 기준으로 한다. `CAD/Print`의 개별 파일은 출력 방향이다. 원래 로봇 좌표에 곧바로 대입하지 않는다.
5. `Data/link_component_body_map.json`/`component_world_bounds_mm.json`과 실제 형상/볼륨/경로를 비교해 객체를 대응시킨다. FreeCAD의 재생성된 Face/Edge 번호나 Fusion entity_token을 영구 식별자로 믿지 않는다. 인스턴스 중복을 이중 질량으로 더하지 않는다.

## 기존 동작 재사용

- 원래 Fusion WaffleExport는 adsk 전용이며 FreeCAD에 그대로 복사해 실행할 수 없다.
- `Motion/native_motion_frames_cm.json`은 원본 보존본이다. 원본 target의 앞3개와 block은cm였다. `Motion/motion_frames_mm.json`은 위치를mm로 변환하고5개 팔 관절각(rad), 집게 벌림(mm), 프레임 번호/시간을 정리했다. `trajectory_mm.csv`도 있다.
- 팔 관절 순서는 BaseYaw, Shoulder, Elbow, WristPitch, WristRoll이다. 그리퍼 모터(J6)의 엔코더 각도가 아니라 aperture로 등가 손가락 이동을 만들었다. 0°/180° 두 끝점만으로 실제 중간 각도-벌림 함수를 선형으로 정하지 않는다.
- `Tools/portable_fk.py`는 표준 Python만으로 URDF의 홈 origin과 관절 운동을 합성한다. 반환 위치는mm다. 기준 고정 변환×관절 변환 순서이며 mimic을 반영한다. 이전 데이터와402프레임 수치 교차검산 결과는 `Evidence/handoff_portability_audit.json`을 본다.
- `Tools/FreeCAD_preview_motion.FCMacro`를 FreeCAD GUI에서 실행하면 **별도의 메시 미리보기 문서**를 만들도록 작성돼 있다. 시작/정지 버튼과12fps 타이머를 제공한다. 임시 퍽은30×30×60mm며 원본시연과 같이 고정방향의 움직이는 박스다. 집게 접촉으로 물리적으로 붙었다고 모델링하지 않았다.
- 이 미리보기는 `Motion/Preview_Meshes_mm`의 링크별 STL을 사용한다. 이는 URDF visual transform을 이미 적용한 링크 좌표 메시(mm)다. STL에 다시SI 0.001 배율을 곱하면 안 된다. 전체 CAD 수정을 메시 미리보기 문서에서 하지 않는다.
- FreeCAD 매크로의 GUI/Qt/Mesh/Placement 동작은 수신 세션에서 실제 시험해야 한다. 수치 FK 검사와 실제 GUI 시험을 구분한다. FreeCAD 내장 Assembly 구속으로의 이식은 별도 작업이다.

## URDF와 후속 시뮬레이션

`ROS/waffle_4wd_description_261021`에는 표시용 메시와 이번에 추가한 물리 시뮬레이션 입력이 있다. Linux ROS 워크스페이스 `src` 아래에 패키지 폴더를 두고 해당 환경에서 빌드/검사한다. 설치된 ROS 환경에서 확인할 예:

```sh
colcon build --packages-select waffle_4wd_description_261021
. install/setup.bash
ros2 launch waffle_4wd_description_261021 display.launch.py
```

이 명령은 이 세션에서 Ubuntu에서 실행한 결과가 아니다. actual ROS배포판/Jetson지원 여부는 해당 환경에서 확인한다. 이름에 Lyrical이라는 이전 사용자 표현만으로 Jetson의 운영체제·ROS 조합이 지원된다고 단정하지 않는다.

FreeCAD에는 ROS2 URDF 생성용 RobotCAD/CROSS 계열 외부 워크벤치가 있으나 현재 패키지의 자동 이동/질량/관절 대응을 보증하지 않는다. 설치 버전의 호환성을 확인하고14물리 링크의 CoM·관성과 집게mimic, 추가9고정 프레임을 포함한23/22 트리를 유지하며 비교한다. STEP만 내보내면 관절/물성이 저절로 URDF로 옮겨진다고 생각하면 안 된다.

Fusion용 내보내기 복사본은 F3D에14개 leaf 링크와13개 관절을 담는다. 기존 원본 관절은 부품1/2 순서와 겹치는 강체그룹/중첩바퀴 때문에 converter의 orphan 오류가 났다. 현재 패키지는 별도 정리 복사본에서 추출 후 집게mimic·물성정밀도·관절범위·effort·충돌·센서TF·footprint를 후처리했다. generic Fusion2URDF 재실행은 이 후처리를 생략할 수 있다.

`Legacy_Fusion_Code/URDF_Postprocess`는 기존 submit/urdf 묶음용 도구의 보존본이다. 현재 CADai 경로에서 그대로 실행한다고 가정하지 말고 ROS/Data 위치에 맞춰야 한다. 구형 finalizer는 새 설정을 지우지 못하도록 실행을 막았다. 과거 원본은 `Pre_Simulation_Baseline`에 보관하며 현재 패키지에 실행하지 않는다. `Tools/portable_fk.py`와 새 FreeCAD 매크로는 CADai 상대 경로를 사용한다. 실제 FreeCAD/ROS/Isaac 환경에서06_SIMULATION_CHANGES.md의 적용 내용을 검증한다.

## F3D/F3Z와 자료 보존

현재 최신 Fusion 원본은 외부참조 없는F3D다. `CAD/Original_Archive/Assembly_261020_Floor249p95.f3z`는 모듈 분할 이전의 과거 상태다. 최신과 같다고 표현하지 않는다. 일부 FreeCAD 확장 도구에는F3D 가져오기 지원 코드가 있지만 F3Z 어셈블리·파라미터·관절을 완전 복구한다고 보증할 수 없다. 따라서 STEP을 무료 CAD 교환 기준으로 쓴다.

확인한 프로젝트/공식 자료(2026-10-21):

- [FreeCAD Assembly 공식 문서 보존본](https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Assembly_Workbench.md):내장 어셈블리/관절 도구와Python 예제. 문서 저장소는보존 상태이므로 실제설치버전 기준으로 다시 확인한다.
- [FreeCAD API](https://www.freecad.org/api/):실행 환경에서 이용 가능한Python/API를 확인하는 출발점.
- [RobotCAD 프로젝트](https://github.com/FreeCADNexus/freecad.robotcad):ROS2 URDF용외부워크벤치. 현재환경설치/호환검증은하지 않았다.
- [InventorLoader F3D 가져오기 코드](https://github.com/jmplonka/InventorLoader/blob/master/importerF3D.py):F3D용코드가 존재한다는 근거. 이 로봇의F3D/F3Z를 완전히 변환했다는 근거가 아니다.
- [Autodesk F3D import API](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/core_ImportManager_createFusionArchiveImportOptions.htm):Fusion 안에서도F3D와F3Z가다른가져오기경로임을 설명한다.

