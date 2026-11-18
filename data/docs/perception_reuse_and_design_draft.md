# 인지 파트: 재사용 후보와 설계 초안 (초안, 2026-10-22)

작성: "인지 파트 코드" 세션. 이 파일은 학습 폴더 밖 공유 노트다. 코드는 아직 없다.
출력 토픽은 팀 약속을 받기 전이라 모두 **미정**이다. 제출 저장소에 이 파일을 그대로 넣지 않는다.

## 1. 재사용 후보

과거 팀 저장소 `~/git/Lv2_Monglian_Assignment`는 이번 팀 저장소가 아니다. 자료로만 쓰고 커밋하지 않는다.
기준은 태그 `lv2-module5-submit`(3619907b7f43)이고 경로는 모두 `lv2_module5/` 아래다. 작성자는 `git log`로 확인했다.

| # | 무엇 | 어디서 (작성자) | 왜 | 고칠 것 |
|---|---|---|---|---|
| 1 | 시리얼 상태 줄 해석, 보드 시각→ROS 시각 변환(`BoardClock`), FAULT 자동 복구(`FaultRetry`) | `ros2_ws/src/tracker_bridge/tracker_bridge/protocol.py` (JuneKunst, onchangbum) | 엔코더 값에 "보드가 읽은 시각"을 stamp로 붙이는 방법이 이미 검증돼 있다. 오도메트리 시각 정확도와 바로 이어진다 | 상태 줄 `S <ms> pan tilt ...`(2축, °)를 **모터 ID 목록 길이만큼 값을 받는 형식**으로 바꾼다. 바퀴는 각도(°) 대신 누적 회전각(rad)과 속도(rad/s)를 쓴다 |
| 2 | OpenCR 시리얼 브리지 노드: 포트 열기, 비차단 줄 읽기, 시리얼 로그 보존(`keep_previous`), 상태 끊김 감시(NO_STATUS), dry_run | `ros2_ws/src/tracker_bridge/tracker_bridge/opencr_bridge.py` (JuneKunst, onchangbum) | 실물 엔코더 입력의 뼈대가 된다 | `pan`·`tilt` 이름 고정을 설정의 바퀴 목록으로 바꾼다. home·B·I 명령(팬·틸트 전용)은 뺀다. **한 시리얼 포트는 한 노드만 열 수 있어서**, 속도 명령 송신(제어 파트)과 상태 수신(인지 파트)이 한 노드 안에 같이 있어야 한다. 그래서 담당 경계를 팀과 정해야 한다 |
| 3 | OpenCR 펌웨어: 100 Hz 루프, 막히지 않는 USB 출력(`LineOut`), 명령 300 ms 타임아웃, DXL 통신 연속 실패 시 FAULT, 50 Hz 상태 줄 | `firmware/opencr_tracker/opencr_tracker.ino` (onchangbum, pizzaafterhangover) | 엔코더를 일정 주기로 읽어 보내는 부분과 안전 정지 구조를 그대로 쓸 수 있다 | `IDS[N]={11,12}`·`N=2` 상수를 바꾼다. 팬·틸트 각도 한계는 바퀴(continuous)에 맞지 않아 뺀다. 한계·변환 계수는 **XM430-W210 기준**으로 새로 확인한다(W350 값을 쓰지 않는다). 펌웨어가 인지 파트 몫인지 확인이 필요하다 |
| 4 | 모터 ID·모델 확인 펌웨어 | `firmware/dxl_scan/`, `firmware/motor_check/` (onchangbum) | 바퀴 모터 4개의 ID·모델·통신 속도를 처음 확인할 때 쓴다 | W210 모델 번호는 e-Manual로 확인해서 넣는다(지금은 확인 전) |
| 5 | launch 인자 덮어쓰기 `override()`, `config_dir:=` 설정 폴더 교체, realsense `rs_launch.py` include(`forwarding=False`) | `ros2_ws/src/tracker_bringup/launch/perception.launch.py` (JuneKunst, onchangbum, SungjinChoi) | 노드 이름 yaml이 `-p` 값을 덮는 문제를 이미 해결해 둔 방식이다. 실험용 설정 사본을 넘기기 좋다 | RealSense 2대(serial_no·camera_name·namespace 구분)를 넣는다. `source:=sim\|real` 인자를 추가한다 |
| 6 | 인지 출력 규칙: 원본 stamp·frame_id 복사, stamp가 늘지 않는 입력 버림, 센서가 멈추면 발행 중단(새 시각을 붙이지 않음) | `ros2_ws/src/target_detector/target_detector/target_detector.py` 64·275~283·348행, `docs/interface.md` 2절 | 다른 파트가 "입력 끊김"을 타임아웃으로 알아챌 수 있게 하는 약속이다 | 그대로 규칙으로 가져온다. 코드는 작은 함수만 가져온다 |
| 7 | 인터페이스 문서 틀(토픽·형식·발행/구독·QoS·주기·필드 단위·부호, 상황별 발행 규칙, 확인 방법) | `docs/interface.md` (팀 문서) | 팀 약속 문서가 없거나 빈칸이 있을 때 이 틀로 제안할 수 있다 | 내용은 이번 과제에 맞게 새로 쓴다 |
| 8 | bag 기록·재생·분석: 모든 토픽을 구독한 뒤부터 시간 세기, 설정 사본, 기준 커밋, sha256 | `scripts/record_bag.sh`, `replay_bag.sh`, `analyze_bag.py` (JuneKunst, onchangbum) | cuVSLAM·Cartographer 같은 조건 비교에 같은 bag 재생이 필요하다 | 토픽 목록을 인자나 파일로 받는다. `lv2_module5` 경로·이름을 바꾼다 |
| 9 | 구독만 하는 헤더·주기·QoS 검사 도구 | `scripts/check_target_headers.py`, `check_target_stream.py` (SungjinChoi) | 약속한 이름·형식·frame_id·주기로 발행되는지 수치로 판정하는 도구로 쓸 수 있다 | 토픽 하나 고정을 약속 목록 전체로 일반화한다 |
| 10 | 정지 계층 설명 형식(인지→제어→브리지→보드→모터) | `config/safety.yaml` (JuneKunst, onchangbum) | 인지 쪽 책임(입력이 멈추면 발행 중단)을 문서로 남기는 틀이다 | 값은 이번 과제 기준으로 새로 정한다 |
| 11 | 공통 값 `/**`와 노드별 항목을 나눈 설정 형식 | `config/camera.yaml` | 센서 목록 설정 파일 구조의 참고 | — |

학습 폴더 `~/git/lv2_next_module`에서 참고할 것:

| 무엇 | 어디서 | 왜 |
|---|---|---|
| 로봇 URDF와 센서 프레임 이름 | `mine/urdf/waffle_4wd_description_261021/urdf/waffle_4wd.urdf`, `sim_ws/src/waffle_4wd_description_261021` | frame_id의 출처다. 바퀴 반지름·위치를 설정 파일 값과 대조한다 |
| Isaac Sim OmniGraph 발행 구성(clock, tf, odom, 스테레오 카메라·CameraInfo, RTX 라이다 LaserScan) | `bridge_check/scene_bridge_check.py` | 시뮬레이션 입력 토픽이 어떤 모양으로 들어오는지 알 수 있다. 그래프 작성은 시뮬레이션 세션 몫이다 |
| 알려진 문제: Lyrical에서 CameraInfoHelper의 K/R/P가 한 칸씩 밀림, Isaac Sim은 Twist·Nav2는 TwistStamped, 큰 영상 BEST_EFFORT 수신에는 `rmem_max` 4 MB가 필요 | STATUS.md 5장 1·3번, HANDOFF_module3.md 4장 | 시뮬레이션 CameraInfo를 직접 발행해야 할 수 있다. 영상 QoS를 정할 근거가 된다 |
| XM430 Protocol 2.0·1 Mbps·속도 모드 설정과 빌드·업로드 기록 | `~/git/physicalai-lv2-OnChangbum/lv2_module1/README.md` (태그 `lv2-module1-submit`) | 바퀴 모터 설정 절차를 참고한다(값은 W350 기준이라 그대로 쓰지 않는다) |

## 2. 확인하다 발견한 설계 문제

1. **URDF의 RealSense 프레임은 하나뿐이다**(`top_camera_{front,rear}_link` → `_optical_frame`). 실물 RealSense는 color·depth·infra1·infra2 광학 프레임이 따로 있고, 스테레오(infra1/2) 사이에 baseline이 있다. cuVSLAM 스테레오 입력과 첫 검사 4번(baseline 확인)을 하려면 프레임 이름과 발행자를 정해야 한다. 방법은 두 가지다. (가) URDF 새 리비전에 infra 프레임을 추가한다. (나) realsense2_camera의 TF만 쓰고 URDF는 `*_link`까지만 쓴다. 같은 TF를 두 노드가 발행하지 않게 해야 한다.
2. URDF에 IMU 링크가 없다. RealSense가 D435i처럼 IMU가 있는 모델이면 쓸지 정해야 한다.
3. Lyrical에 없는 것: Arducam 드라이버(`v4l2_camera`·`usb_cam` 미설치), LDS 드라이버, `dynamixel_sdk`, `laser_filters`, `pointcloud_to_laserscan`, `camera_calibration`. 있는 것: `realsense2_camera`, `cv_bridge`, `image_transport`, `laser_geometry`, `message_filters`, `tf2_ros`, `diagnostic_updater`, `robot_localization`, `isaac_ros_cuvslam`, `isaac_ros_image_proc`.
4. 라이다 2개를 하나로 합치는 패키지(ira_laser_tools 등)가 Lyrical에 없다. Cartographer는 LaserScan 여러 개를 직접 받을 수 있다(`num_laser_scans`). 그래서 합칠지는 팀 약속(`/scan` 하나인지 둘인지)에 따라 정한다.

## 3. 설계 초안

### 노드 구성 (패키지 이름·언어는 미정, 가칭 `perception`)

| 노드 | 역할 | 입력 출처 |
|---|---|---|
| `wheel_state_bridge` | OpenCR 상태 줄 → 바퀴 JointState. 이름과 순서는 `wheels.yaml` 목록을 따르고, stamp는 보드가 읽은 시각이다 | 실물 |
| `joint_state_relay` | Isaac Sim JointState에서 바퀴 관절만 골라 `wheels.yaml` 이름·순서·부호로 맞춘다 | 시뮬레이션 |
| `wheel_odometry` (담당 확인 필요) | 바퀴 JointState → 오도메트리(`odom→base_footprint`). 차동 구동 계산은 별도 모듈 하나에만 둔다 | 공통 |
| `scan_relay` / `scan_merger` (약속에 따라) | 라이다 목록마다 frame_id·stamp 확인 후 재발행. 또는 `laser_geometry`·`tf2`로 합쳐 하나로 발행 | 공통 |
| `camera_relay` (필요하면) | 카메라 목록마다 이름 재매핑, frame_id·stamp 확인. 시뮬레이션 CameraInfo는 직접 발행(K/R/P 밀림 우회) | 공통 |
| `sensor_monitor` (선택) | 약속 목록 전체의 주기·stamp 증가·frame_id를 검사해 diagnostics로 발행 | 공통 |

카메라·라이다 드라이버 자체(realsense2_camera 등)는 launch에서 실행만 한다. 드라이버 노드가 곧바로 약속 이름으로 발행할 수 있으면(재매핑) relay 노드는 만들지 않는다. 지연과 복사를 줄이기 위해서다.

### 입력 대응표 (출력은 미정)

| 센서 | 실물 입력 | 시뮬레이션 입력 | frame_id (URDF) | 출력 토픽 |
|---|---|---|---|---|
| RealSense 전 | realsense2_camera(color, depth, infra1/2, camera_info) | Isaac Sim 카메라 + 직접 발행 CameraInfo | `top_camera_front_optical_frame` (infra는 2절 1번) | 미정 |
| RealSense 후 | 같음(serial_no로 구분) | 같음 | `top_camera_rear_optical_frame` | 미정 |
| Arducam (팔 끝) | 드라이버 미정(UVC 여부 확인) | Isaac Sim 카메라 | `camera_optical_frame` | 미정 |
| 라이다 전 | LDS 드라이버(모델 확인 필요) | Isaac Sim RTX 라이다 LaserScan | `lidar_front_scan` | 미정 |
| 라이다 후 | 같음 | 같음 | `lidar_rear_scan` | 미정 |
| 라이다 3번째 | 실물만 있음 | 없음(시뮬레이션은 2개로 가정) | URDF에 없음 | 미정 |
| 바퀴 엔코더 4 | OpenCR 시리얼 상태 줄 | Isaac Sim JointState(`Wheel_Revolute_*`) | (JointState) / 오도메트리는 `odom`→`base_footprint` | 미정 |
| 팔·집게 엔코더 6 (범위 확인 필요) | OpenCR(같은 보드인지 확인) | Isaac Sim JointState(`RMX52_*`, `gripper_*`) | (JointState) | 미정 |

### 설정 파일

- `wheels.yaml`: 바퀴 목록. 항목마다 `name`, `joint`(URDF 관절 이름), `motor_id`, `radius_m`, `position_xy_m`, `direction`(+1/−1)을 둔다. 실물과 시뮬레이션이 같은 파일을 쓴다. 좌우 구분은 이름이 아니라 `position_xy_m`의 y 부호로 계산한다. 값은 URDF와 대조해서 넣는다(반지름 33 mm, 좌우 간격 279.2 mm, 앞뒤 간격 160 mm). 모터 ID는 dxl_scan으로 확인한 뒤 넣는다.
- `sensors.yaml`: 카메라·라이다 목록. 항목마다 입력 토픽(sim/real), 출력 토픽, frame_id, 기대 주기, QoS를 둔다.
- `interface.yaml` 또는 문서: 팀 약속(이름·형식·QoS·단위·부호)을 그대로 옮긴 기준표. 9번 검사 도구가 이 파일을 읽어 판정한다.
- launch: `perception.launch.py source:=sim|real config_dir:=<폴더> run_id:=<이름>`. 인자 값은 `override()` 방식으로 넘긴다.
