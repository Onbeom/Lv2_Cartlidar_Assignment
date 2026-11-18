# 인지 코드 및 과제 연동 검토 — 2026-10-26

## 범위와 검증

- `Lv2_Cartlidar_Assignment`의 현재 작업 파일(커밋되지 않은 바코드 변경 포함), `lv2_next_module`의 RL1 학습 환경·기록, Nav2 실행 파일을 검토했다.
- 첨부 이미지 4개와 ZIP의 목록·최상위 설명문을 확인했다. ZIP은 주로 미로 제작 CAD/STL·조립 자료이며, 내부 조립 지시를 코드 수정 요청으로 취급하지 않았다.
- 과제 규칙은 사용자가 설명한 빨강·초록 × 직육면체·원기둥의 4종을 우선했다. 이미지의 파란 퍽 ID 8~11은 현재 사용할 물체인지 별도 확인이 필요하다.
- 소스 코드는 수정하지 않았다. 기존 사용자 변경을 유지했다.
- ROS 2 Lyrical에서 `robot_interfaces`, `perception` 새 빌드 성공. TensorRT/CUDA 경로도 빌드에 포함됐다.
- 인지 단위 시험 85개, 실패 0. 결과: `/tmp/cartlidar-review-build/perception/test_results/perception/test_perception_logic.gtest.xml`.
- 실센서, Jetson, 실제 YOLO 엔진 추론, Isaac Sim에서 전체 미션 수행은 실행하지 않았다. 단위 시험 통과는 이 통합 동작의 성공을 뜻하지 않는다.

## 확인한 문제

### 1. [P1] 시작 미션의 장소 태그 12를 읽을 수 없다

위치: `perception/src/perception_node.cpp:1356`, `perception/config/perception.yaml:71`, `center/src/mission_node.cpp:394`, `center/config/mission.yaml`.

시작 상태는 READ_TAG를 켜고 `PerceptionResult.tag_id`를 기다린다. 그러나 READ_TAG는 `tag.ids`의 0~11만 남긴다. 미션표의 `tag_12`는 여기에 도달하지 못한다. 장소 태그 12~14는 별도 READ_MARKER 경로로 읽지만 center는 이 결과를 구독하지 않는다.

개선: 장소 태그를 시작 명령으로 쓸 경우 READ_MARKER와 TaskMarker 구독을 center에 연결한다. 48 mm 장소 태그를 16 mm 퍽 태그 설정에 단순 추가하면 자세의 거리 축척이 틀려지므로 두 검출 경로는 유지해야 한다. 퍽 태그 7을 시작 명령으로 쓸 의도인지도 확정해야 한다.

### 2. [P1] 학습 정책과 배포 제어부의 입력·출력이 다르다

위치: `robot_policy/config/policies.yaml:5`, `control/src/manipulator_node.cpp:716`, `lv2_next_module/261025_2117/rl1_arm_reach/waffle_rl/reach_env_cfg.py:90`, `waffle_rl/robot_cfg.py`.

| 항목 | Isaac Lab RL1 | 현재 pick_cup_v1 배포 설정 |
|---|---|---|
| 관측 | 관절 상대 위치 4 + 상대 속도 4 + 목표 위치 3 + 이전 행동 4 = 15 | 관절 위치 6 + 영상 오차·valid 4 + 거리·valid 2 = 12 |
| 행동 | 팔 4관절, 버전별 속도 명령 또는 정규화 위치 명령 | 6관절 절대 위치 rad |
| 주기 | 30 Hz | 20 Hz |
| 관절 이름 | RMX52_* | joint1~joint5, gripper_roll |

학습 로그 `results/logs/train_v3s3_seed1.log:1149`도 관측 15차원을 확인한다. 현재 로더는 차원 불일치를 거부하므로 RL1 ONNX를 이름만 바꿔 배포할 수 없다. 차원 숫자만 바꿔도 상대 관절 위치, 이전 행동, 목표 좌표, 행동 변환이 다르므로 해결되지 않는다.

개선: 관절 순서·단위·영점, 15개 관측 생성, 4개 행동의 변환·적분·속도 제한, 주기를 학습과 일치시키는 배포 어댑터가 필요하다. 나머지 관절과 그리퍼 동작도 별도 정의해야 한다. RL1은 팔 위치 도달 학습이며 집기·운반·놓기 성공 자체를 학습한 환경은 아니다.

### 3. [P1] 새 과제의 미션 흐름이 판단부에 연결되지 않았다

위치: `center/include/center/mission_table.hpp:27`, `center/src/mission_node.cpp`, `center/config/mission.yaml`.

미션 문법은 navigate/search/approach/pick/pose/skill만 지원한다. 모드 문법에도 READ_MARKER, READ_SIGNAL, READ_BARCODE가 없다. center는 SignalSequence·Barcode를 구독하지 않고 CheckDice를 호출하지 않는다. 따라서 인지에서 12회 집계·문자열·배치 판정이 가능해도 점등 횟수를 목표 주사위로 바꾸어 집고 놓는 전체 미션은 진행하지 못한다.

개선: `바코드 해석 → 신호등 집계 저장 → 색별/모양별 재고와 목표 눈 생성 → SLAM/Nav2 이동 → 선택한 퍽 집기·놓기 → 재관측/CheckDice → 재시도` 흐름을 판단 담당과 연결해야 한다. 신호등 모드를 끄면 인지의 집계가 초기화되므로 확정 결과는 판단부가 먼저 저장해야 한다.

추가로 `control/config/manipulator.yaml:45`는 그리퍼가 비활성화되어 있고, 그리퍼 없이 집었다고 가정하는 벤치 설정이다. 현재 설정의 pick 성공을 실물 Pick and place 성공으로 볼 수 없다.

### 4. [P1] 통합 bringup이 시뮬레이션 시계를 전달하지 않는다

위치: `bringup/launch/bringup.launch.py:40`, `:64`.

개별 perception.launch.py는 use_sim_time을 지원하지만, 상위 bringup은 이를 선언·전달하지 않으며 center/control/EKF에도 적용하지 않는다. Isaac Sim의 시각으로 발행된 영상을 기본 통합 실행에 넣으면 시스템 시각과 달라 `frame_max_age_s=0.3` 검사에서 오래된 영상으로 버릴 수 있다. 별도 실행에서 시계를 이미 설정했다면 그 실행에는 해당하지 않는다.

개선: 상위 launch에서 use_sim_time을 선언하고 모든 관련 노드 및 하위 launch에 전달한다. 시뮬레이터의 /clock 발행도 확인한다.

### 5. [P2] 가까운 서로 다른 퍽이 병합에서 사라질 수 있다

위치: `perception/src/perception_node.cpp:903`, `perception/config/perception.yaml:82`.

태그 퍽을 먼저 넣은 뒤, 그 중심에서 40 mm 이내인 모든 D435 퍽을 제거한다. 퍽 폭은 30 mm이므로 태그를 읽은 A와 중심 간격 35 mm인 별도 B도 삭제된다. 두 관측의 색을 비교하거나 일대일 대응을 제한하지 않는다. 반대로 D435 전·후방 간 중복은 이 경로에서 제거하지 않는다.

개선: 촬영 시각·좌표를 맞춘 뒤 색·태그·투영 영역·거리 조건으로 일대일 대응을 구한다. 임계값만 줄이면 센서 위치 오차에 따른 동일 퍽 중복이 남을 수 있다.

### 6. [P2] 팔 정렬이 목표 퍽을 유지하지 않는다

위치: `perception/src/perception_node.cpp:1339`.

ARM_ALIGN은 매 프레임 점수가 가장 높은 검출을 선택한다. D435 추적 목표, 선택한 태그 ID, 색, 직전 영상 위치와의 연결이 없다. 여러 퍽이 함께 보이면 점수 순서가 바뀔 때 다른 퍽으로 정렬될 수 있다. 현재 target_class=-1 설정에서는 색도 고정되지 않는다.

개선: 판단부가 선택한 ID/목표를 전달하고 카메라 투영·태그·추적으로 동일 퍽을 유지한다. 목표가 사라졌을 때 다른 퍽을 즉시 대신 잡지 않도록 실패/재탐색 동작도 정의한다.

## YOLO 사용 시 추가로 고칠 조건부 문제

### 7. [P2] 4개 YOLO 클래스는 현재 메시지 변환과 맞지 않는다

위치: `perception/src/perception_node.cpp:646`.

현재 class 0→빨강, 1→초록, 2→파랑, 나머지→모름으로 변환하고 shape은 항상 UNKNOWN이다. 이는 색상 3클래스 규칙이다. 네 가지 퍽 종류를 4개 클래스로 학습하면 적어도 일부 색이 틀리거나 모름이 되고 형태 정보는 사라진다. 2개 색상 클래스 모델이라면 이 문제의 색 매핑 부분은 해당하지 않지만 형태는 여전히 태그 경로가 필요하다.

개선: 학습 data.yaml의 names와 일치하는 명시적 `class_id → (color, shape)` 매핑을 설정한다. 현재 `detector.engine_path`는 비어 있어 기본 검출은 HSV이며 YOLO 추론이 아니다. 모델 학습·내보내기·실제 엔진 평가 결과까지 확인해야 한다.

### 8. [P2] TensorRT 엔진 형식 검사가 불충분하다

위치: `perception/src/trt_detector.cpp:61`.

고정 float32 NCHW, batch 1, 출력 [1,4+nc,N]을 전제로 메모리를 할당하지만 rank와 출력 채널 하한만 검사한다. dtype, batch/channel 수, 음수 동적 차원을 검증하지 않는다. 동적 입력은 shape 설정도 하지 않는다. FP16 입출력 또는 다른 출력 레이아웃 엔진을 넣으면 잘못된 버퍼 해석이나 할당 실패가 가능하다. FP16 연산을 사용하는 엔진이라도 입출력이 float32면 dtype 문제에는 해당하지 않는다.

개선: 지원 엔진 형식을 명확하게 검사해 거절하거나 동적 shape와 dtype별 버퍼 처리를 구현한다. CUDA 복사·동기화 반환값 검사도 필요하다. 실제 엔진 추론은 이번 검증에 포함하지 않았다.

## 과제 조건 현황

| 조건 | 확인한 근거 | 남은 일 |
|---|---|---|
| Isaac Sim + Isaac Lab AI | USD/PhysX 코드, RL1 환경·학습 체크포인트·로그 존재 | 과제 수행 정책 및 실제 배포 연결 검증 |
| Tag + Barcode CV | 두 검출 경로 구현, 단위 시험 통과 | 장소 태그와 바코드를 판단 미션에 연결 |
| SLAM + Navigation | next_module의 Cartographer 실험 결과, Nav2 launch 존재 | 현재 로봇 TF·토픽·제어 출력과 통합 실행 검증 |
| Manipulator Pick and place | IK·Pick·팔 자세 코드 존재 | 실그리퍼, 퍽 선택, 배치 좌표별 놓기와 성공 검증 |
| 신호등 미션 | 12회 집계와 주사위 판정 함수 존재 | 실제 점등 규칙과 일치시킨 집계, 결과 저장·배치 실행 |

## 추가 확인이 필요한 과제 규칙

1. 12회 점등은 한 번만 재생되는가, 동일 순서가 반복되는가? 현재 confirm_cycles=2이며 신호등 위치를 고정할 때도 3회 점멸 관측을 요구한다. 1회만 재생된다면 시작 동기화와 집계 방식을 바꿔야 한다.
2. 각 색의 횟수는 1~6으로 보장되는가? 0회나 7회 이상, 보유 퍽 수보다 많은 값이 가능한지, 노랑 배치에 쓸 퍽을 빨강·초록 배치와 동시에 확보해야 하는지 확인이 필요하다.
3. 각 색×형태별 개수, 실제 태그 ID 배정, 태그 부착 면은 무엇인가? 이미지의 B1~B4는 사용자의 빨강·초록 4종 규칙과 대조가 필요하다.
4. CODE128 P2-01~04의 명령 의미, 주사위 배치 장소·칸 간격·허용 오차, 신호등 켜짐/꺼짐/묶음 간격을 확정해야 한다.

우선순위: 태그·시계 연결 수정 → 학습/배포 계약 통일 → 신호등·바코드·배치 미션 연결 → 다중 퍽 선택/병합 개선 → 실제 엔진 및 Isaac Sim 전체 미션 검증.
