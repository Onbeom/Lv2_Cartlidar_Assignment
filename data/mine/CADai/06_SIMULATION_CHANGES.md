# 적용한 URDF 변경과 다음 세션의 검증 범위

기존 submit/urdf/waffle_4wd_description_261021 패키지를 직접 갱신했다. CAD 형상·재료·14물리 링크의 질량/CoM/관성은 유지했다. Fusion 원본의 관절 개수와 URDF 논리 프레임 개수는 다르다.

| 항목 | 적용 내용 | 아직 필요한 검증 |
|---|---|---|
| 트리 |23링크22관절, root base_footprint;14물리 링크+9무질량 고정 프레임 | ROS xacro/check_urdf/TF 및 Isaac importer 실행 |
| 지면 |기존 base_link를 base_footprint보다 약6.7499816754mm 위에 둠 | 실제 타이어 접지·변형; Data의 world 좌표는 기존 engineering base_link 기준 |
| 팔5관절 |revolute, yaw±π/shoulder±1.5/elbow−1.4..1.5/pitch−1.97..1.7/roll±π(rad) | 하드 스톱·엔코더 영점·배선 여유 실측; 안전 범위라고 단정 금지 |
| effort |팔3800N·mm,바퀴2700N·mm, URDF내부3.8/2.7N·m |11.1V 제조사 순간 stall 기준; 연속 토크·열·전류·제어기는 별도 |
| 집게 |16.8..72mm, right=−left mimic, 등가힘10N/손가락 |10N은 임시값; 크랭크 구동비·파지력·각도별 개폐 곡선 미측정 |
| 충돌 |바퀴 cylinder, 구조물/팔 단순 box, 스토리지/레일/브라켓/집게 convex STL | PhysX 접촉·필터·관통/근사 오차;1mm는 분해 목표이며 hull 수 제한으로 달성 인증 아님 |
| 센서 |UC684 및 전/후 상단 카메라 optical frame, 전후 LDS scan frame | CAD 기준값; 실제 광학 중심·LDS 스캔 높이와 yaw·캘리브레이션 검증 |
| 동작 |402프레임 원본과 portable FK/FreeCAD 미리보기 보존 | 일부 shoulder 자세가 새 임시1.5rad 범위를 벗어남; Motion/reference_limit_violations.json 확인 후 재계획 또는 실제 한계 확인 |

OpenCR1/2는 각각 별도 터틀봇 공칭11.1V 배터리로 구동한다. Jetson은 별도12V 전원이다. 팔 XM430-W350,바퀴 XM430-W210이 기본 모터라는 사용자 확인을 적용했다. 분리된 제어기 전원과 통신 접지/배선·전류 용량은 실제 배선도에서 확인한다.

카메라 optical frame은 x오른쪽/y아래/z광축 전방이다. UC684는 렌즈 전면 CAD 중심, 상단 카메라는 스테레오 렌즈 CAD 중점이 기준이며 보정된 RGB 광학 중심과 같다고 단정하지 않는다. LDS Z=60.85mm는 기존 가정 스캔 밴드54.7..67mm의 중간값이다.

ROS/Isaac/FreeCAD GUI는 이 세션에서 실행하지 않았다. 정적 XML·단위·질량/관성·메시·축 검사는 런타임 성공이나 실제 제작 성공을 대신하지 않는다. 과거 G채널 이산 충돌 검사는 기존 시연에 대한 결과이며 새 bounded URDF에서 그대로 실행 가능하다는 뜻이 아니다.

제조사 근거: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/ , https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
