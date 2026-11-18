# 새 모델의 인지 좌표계 호환성 점검 · 2026-10-29

원본 `261029_1030/urdf_261028/`는 읽기만 한다. 팀 저장소/모델 생성파일 변경 없음. 모터 제어·경로 계획이 아니라 인지 입력의 TF·센서 메타데이터 계약을 확인한다.

| 확인 | 예상(실행 전) | 결과 |
|---|---|---|
| URDF 링크/관절/메시 참조와 센서프레임 존재 | 루트base_link·팔카메라arm_camera_optical_frame 확인, 센서7개카메라프레임/메시누락0 목표 | 미실행 |
| URDF FK와sensors.json의body기준센서변환 | 7카메라 모두위치차≤2µm·회전차≤5µrad 목표. JSON소수6자리반올림허용, 실제카메라보정값검증아님 | 미실행 |
| 드라이브/접힘/영자세의카메라TF | 서로독립구현Robot FK와순수URDF FK의위치차≤1nm·회전차≤0.1µrad 목표. 원점루트변경을명시하고카메라왕복변환확인 | 미실행 |
| 기존인지도구가정 | base_footprint를조상으로가정한fixed_transform 및RMX52관절명·camera_optical_frame은새모델과불일치예상. 조용히영자세로대체하지않도록별도어댑터에서잘못된관절명거부 | 미실행 |

D435 RGB/IR의실제RTX시야·근거리클리핑대조는YOLO/SLAM세션이261029_2125/에서담당. 이쪽은중복GPU실행안함. 실물SSH/내부보정은사용자사정으로중단 상태.

정적결과:루트base_link·링크42·관절41·메시누락0.카메라7/7메타데이터일치,3자세×7카메라21/21독립FK일치. 위치최대0.6263µm·회전1.2131µrad. 원본frame_audit.json·perception_model_profile.json. 기존fixed_transform(root=base_footprint)은KeyError(base_link),옛RMX52관절명과camera_optical_frame은새모델에없음.

| ROS TF 전달 확인 | 예상(실행 전) | 결과 |
|---|---|---|
| PC domain96 LOCALHOST의robot_state_publisher+정적joint_state입력,3자세×7카메라×2기준프레임42변환 | 42/42위치차≤10nm·회전행렬최대요소차≤1e-7 목표. /cmd_vel·모터명령없이TF출력계약만검사 | 미실행 |

| 인지 노드 연결 합성 시험 | 예상(실행 전) | 결과 |
|---|---|---|
| 새URDF의drive관절상태·odom→base_link·D435640×480·팔1280×720 합성빨강원영상/깊이, odom2자세 | 안정화후10프레임 모두object위치오차≤30mm·arm offset오차≤0.05·유효 목표. 새렌더/실물정확도가아니라루트/프레임/종횡비연결검사 | 미실행 |

## 새 모델 인지 TF 점검 결과 · 2026-10-29

원본261029_1030/urdf_261028/의AGENTS·CLAUDE·REVISION을읽고생성파일수정없이점검. 입력주요5파일SHA실행전후동일(source_unchanged.json). ROS루트base_link,카메라3대의RGB/IR합계7광학프레임.

- 정적:메시참조누락0,센서메타데이터7/7일치(위치차최대0.6263µm·회전1.2131µrad),3자세21/21독립FK일치.
- 실제PC ROS TF시험:domain96·LOCALHOST·/joint_states합성입력만사용,3자세×7광학프레임×2기준프레임42/42일치. 모터명령없음. RSP의루트관성KDL경고는기록했고TF일치에는영향없음.
- 인지노드연결:별도domain90에서새URDF+odom→base_link+drive관절상태+D435640×480/팔1280×720합성영상. odom2자세에서안정화후10/10정렬값유효·목표위치오차0.47035mm·화면오차0.00388797. 렌더/실물/A1성능판정이아닌입출력계약시험임.
- 옛재생기처럼odom→base_footprint를발행하면새URDF의base_link→base_footprint와부모관계가충돌할수있으므로신규재생은odom→base_link로변경. 기존fixed_transform의base_footprint조상가정도새모델에서KeyError발생. 기존RMX52_*·camera_optical_frame이름은사용불가. 잘못된관절명을거부하는별도ModelFrames어댑터를준비함.

결과 `Cartlidar/sim_render/results/new_model_261029/`:frame_audit.json·ros_tf_summary.json·perception_protocol_summary.json·perception_model_profile.json. 도구 `261029_new_model_audit/`,사진`photo/261029_새모델_카메라좌표계.png`. 사진의drive/stow/zero는정적좌표계도식이며모터움직임/가시성검증아님.

실물보정:추가SSH없음.이미수신한Arducam미리보기1920×1080한장을오프라인검출했으나마커0·코너0. 이사진으로K/D를구하지않음. 재개시화면번짐/초점·노출및보정판검출부터확인할것. 실제칸21mm·마커15.12mm기록유지.

다음분담:YOLO/SLAM세션이새fullUSD의RGB/IR클리핑/원시깊이/자기가림렌더를맡음. 여기서는중복GPU실행하지않았음. 신규렌더가확인되어도실물K/D와발제배치의A1정식재판정은별도이며이전모델수치를새모델수치로옮기지않는다. 팀코드반영/commit/push/PR없음,본인ROS프로세스종료.

### 다른 세션의 새 모델 입력 점검 수신·문서 대조

`codex/2026-10-29_새모델_카메라_검증.md`를직접읽어이세션TF점검과출처구분. 상대세션은새fullUSD지정HFOV/near105mm조건에서6/6 RGB/IR시야의깊이유효≥90%·중앙≥.105m,기준자세TF의nvblox90입력도구연결을보고. 실제카메라K/D·주행SLAM·R계열성능재판정이아님. 광학원점은메시안쪽이며near로제외하는문서상조건,실물깊이+8mm금지유지. 이세션은해당GPU/nvblox시험을구현·실행한것이아님. 서로의개인결과경로와새루트odom→base_link계약을공유완료.
