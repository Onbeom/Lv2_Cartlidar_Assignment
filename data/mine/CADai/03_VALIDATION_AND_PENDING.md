# 검증 근거와 미완료 항목

아래 과거 CAD 검사는 이 인수인계를 만들며 모두 다시 실행한 것이 아니다. `Evidence`의 기존 결과를 인용했다. 이번 묶음 생성 때 재검사한 범위는 파일 해시/압축 무결성, 이식한 FK의402자세 비교, 메시 단위/외곽, 구문 및 데이터 연결이다. FreeCAD GUI·물리 시험을 수행한 것으로 표현하지 말 것.

## 기존 검사 결과

| 항목 | 결과 | 근거와 범위 |
|---|---|---|
| 최종 어셈블리/GeometryOnly 형상 일치 | 통과 | modular_review3_independent.json:669개 형상 signature 및 질량 비교 |
| 분할 브라켓 수 | 4개 | modular_review2_reopen.json, modular_bracket_built.json |
| 판넬 체결 구멍 중심 정렬 | 통과 | modular_panel_alignment.json, 좌우 각 층9개 중심, 최대 약0.000023mm |
| 모터/프로파일/분할부 CAD 간섭·장착 검사 | 해당 검사 범위 통과 | modular_bracket_validation.json과 개별 fit/relief 자료; 접촉은 의도된 부분이 있음 |
| STL 폐곡면/비다양체/256×256mm 배드 외곽 | 통과 | modular_stl_audit.json; 실제 슬라이싱/출력 성공 아님 |
| 브라켓 메시 관성과 CAD 관성 비교 | 통과 | modular_review2_independent.json; 전자부품 물성의 실측 검증 아님 |
| 402자세에서 정해진 정적 장애물 충돌 | 이산 검사 통과 | modular_motion_collision.json; 연속 sweep/팔 전체 self collision/12개 파지 과정 제외 |
| 원래 Fusion 시연 재생/기본자세 복원 | 완료 기록 | modular_final_reopen_verified.json 및 player_status.json 참고; 새 FreeCAD 매크로의 실행 결과 아님 |
| 12개 적재 정적 지지 다각형 | 이산 계산상 내부 | MAX_12_PAYLOAD_AUDIT.json:최소40.91422897mm; 동적 안정성/13번째 퍽 없음 |
| 중력토크 수치 교차검산 | 통과 | modular_review3_independent.json:위치에너지 유한차분과 외적 계산 비교 |
| LDS 바깥180° CAD 가림 | 가정 범위 통과 | modular_lidar_fov.json:스캔 높이54.7~67mm 가정; 실측광학 평면 아님 |
| 접힌 외곽 치수 | 한 시험 자세 통과 | modular_fold_measurement.json; 전후444.49×좌우305.2mm, 전역 최솟값 아님 |
| URDF 단일 트리/참조 메시/CoM/관성/단위/축 | 통과 | 기존 urdf_independent_audit.json은14/13 기준; 이번 simulation_audit.json은23/22와 기존14물리 링크의 물성/표시 형상 보존을 검사 |
| 변환용 F3D 저장 후 다시 열기 | 통과 | modular_urdf_archive_reopen.json:14/13/669 및4891.194g 유지 |
| 3회 독립 검토 | 완료 기록 | review_pass_state.json; notes의 '최종 ROS package 미검증' 중 구조/수치 부분은 이후 URDF 감사가 보충함. ROS 런타임은 여전히 미검증 |

## 추가 검증에 필요한 데이터

| 미완료 항목 | 아직 완료하지 않은 이유 | 필요한 입력/다음 행동 |
|---|---|---|
| PLA 브라켓 강성·피로·크리프·나사부 파손 | 실제 출력 이방성/체결력/하중 조건 없음 | 출력 방향·재료/인필/벽수·탄성/강도·토크·가감속/충격·안전율, 해석 또는 실물 하중 시험 |
| 실제 출력 성공/서포트 제거 | slicer/프린터 실험을 하지 않음 | 실제 Bambu 프로파일·노즐/적층·서포트·브림·냉각/수축, 슬라이싱 검토와 시험 출력 |
| 소켓 끼움/슬롯 너트/M5 체결 확실성 | 공차/너트 형상/마찰/조임 실측 없음 | 실제20×20 프로파일 홈 도면·너트·볼트 머리/길이·출력 공차·체결 시험 |
| 모터 연속 토크/열/전류/속도 | 준정적 중력토크만 계산 | 모터 모델과11.1V는 확인됨. 연속 허용 운전 곡선·감속기·팔 자세/속도/가속/듀티·온도·전원 |
| 관절 영점·한계 | 원래 모터/실제 스토퍼와 CAD 영점 대응 미확인 | 실물 엔코더 영점, ±방향, 하드 스톱/배선 사진 또는 측정 |
| 집게 내부 크랭크/파지력 | 최소/최대 벌림과 일부 각도 관찰만 있음 | 링크/축 위치·기어비·각도별벌림 측정·패드 마찰·물체실측질량·구동토크 |
| 모든12개 투입/회수/연속 충돌 | 현재 G채널 한 시연의 이산 경로 | 채널/적재층별 목표, 물체 접촉/부착 상태, 자세/시간, 연속충돌 및 self collision |
| 동적 전복/슬립 | 평지 정적 지지 다각형 가정 | 바퀴 접지/마찰·경사·서스펜션·가감속/제동·팔운동·적재 변화 |
| 센서 성능/카메라 캘리브레이션 | CAD 배치만 검토 | UC-684/D435 내부·외부 파라미터/초점·시야, LDS 실제 스캔 평면·커버/전선 가림 |
| ROS/RViz 실행 | 해당 Ubuntu/Jetson 런타임에서 실행하지 않음 | 실제 배포판·설치된ROS와패키지, check_urdf/xacro/colcon/TF 및 표시 테스트 |
| Isaac Sim/Lab 물리 환경 | 이번에 단순 primitive와 convex STL을 추가했지만 실제 PhysX/articulation 설정 미검증 | 사용 버전·convex decomposition·마찰/접촉·joint drive/effort/속도·하드웨어 대응 |
| FreeCAD native 어셈블리/매크로 실행 | 이 세션의 FreeCAD 런타임이 없음 | 실제FreeCAD버전/GUI/매크로 실행, STEP 가져오기 인스턴스/배치 확인, FCStd 구속 재구성 |
| 최종 실측 물성 | 형상/밀도와 합의 질량의 추정뿐 | 인필/벽·부품 무게·실측CoM·부가체결재/전선, 필요한 경우 흔들림/관성 측정 |

설계를 수정한 뒤 이전 검사 결과를 그대로 현재 결과로 쓰지 않는다. 판넬·모터 장착점, 형상 간섭, 기본/접힌 외곽, 출력성, 질량·CoM·관성, 중력토크·적재 및 경로를 변경 영향에 맞게 다시 검증한다. 실험 자료가 필요한 항목은 미완료로 유지한다.

