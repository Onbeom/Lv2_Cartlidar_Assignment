# 인지 파트 코드 재사용 기록 (report용)

작성: 2026-10-22. 대상 브랜치: `Lv2_Cartlidar_Assignment` `feat/perception-sim-input`(dev `0f00432`에서 시작).

## 1. 이전 모듈 팀 코드 (Lv2_Monglian_Assignment, 태그 `lv2-module5-submit` = 3619907b7f43)

원본은 Python이라 그대로 복사하지 않았다. 규칙과 로직만 C++로 다시 썼고, launch 함수 하나만 그대로 가져왔다.

| 무엇 (원본 위치 · 작성자) | 왜 | 이번에 쓴 곳 | 고친 점 |
|---|---|---|---|
| 영상 stamp 규칙: 같은·과거 stamp 버림, 1 s 넘게 뒤로 가면 시각 재시작으로 보고 상태 초기화 (`lv2_module5/ros2_ws/src/target_detector/target_detector/target_detector.py` `on_color`, 275~283행 · target_detector 작성자 JuneKunst, Ohn_Changbum, onchangbum, SungjinChoi) | 재전송 영상으로 결과를 두 번 내지 않게 한다. 시뮬레이션 리셋이나 bag 반복 재생 뒤에 영상을 모두 버리는 일도 막는다 | `perception/include/perception/frame_gate.hpp` `StampGate`, `perception_node.cpp` `gate_frame` | Python → C++. 카메라마다 따로 두고, 재시작 때 깊이 대기열과 추적 상태를 지운다 |
| 컬러와 stamp가 가장 가까운 깊이 찾기, 허용 차이 밖이면 없음 (`target_detector.py` `find_depth`, 261~272행) | 깊이를 기다리느라 컬러 처리가 멈추지 않게 한다 | `frame_gate.hpp` `NearestCache`, `perception_node.cpp` `take` | Python → C++ 템플릿, 용량 5 |
| 깊이 인코딩별 배율(16UC1 = mm, 32FC1 = m) (`target_detector.py` 271행) | Isaac Sim 깊이(32FC1)와 realsense2_camera 깊이(16UC1)를 같은 코드로 받는다 | `geometry.hpp` `depth_scale_for_encoding`, `median_depth`(CV_32F 추가) | C++. NaN·inf 제외를 추가했다 |
| 깊이 짝 허용 0.02 s와 그 실측 근거 (`lv2_module5/config/camera.yaml` `depth_sync_tol_s`) | 값의 근거 | `config/perception.yaml` `depth.sync_tol_s` | 없음 |
| launch 인자를 노드 이름 항목의 임시 yaml로 넘기는 `override()` (`lv2_module5/ros2_ws/src/tracker_bringup/launch/perception.launch.py` · JuneKunst, onchangbum, SungjinChoi) | dict·`-p`로 넘긴 값이 설정 yaml에 덮이는 문제를 피한다 | `perception/launch/perception.launch.py` | 함수는 그대로 가져왔다. 넘기는 값은 `use_sim_time` |
| "카메라가 멈추면 발행하지 않는다(침묵)" 규칙 (`lv2_module5/docs/interface.md` 2절) | 받는 쪽이 끊김을 시각 규칙으로 알게 한다 | `perception_node.cpp` `process` | 새 영상이 없으면 같은 결과를 다시 내지 않는다는 조건을 추가했다 |

## 2. 이번 팀 저장소의 기존 코드 (kanichong, `eaaca14`)

사용자 허가(2026-10-22)를 받고 고쳤다. 팀 코드는 앞으로 전면 수정될 예정이다.

| 파일 | 고친 점 | 이유 |
|---|---|---|
| `src/perception_node.cpp` | D435 SDK(librealsense2) 직접 열기와 Arducam V4L2 직접 열기를 빼고, 영상 토픽 구독(켜진 모드의 카메라만)으로 바꿨다. 스레드 3개와 busy loop 대신 ROS 타이머와 콜백 그룹을 쓴다. 깊이 점은 영상 header의 컬러 광학 프레임에서 TF로 바로 바꾼다(기존 `optical_to_link` + `camera_front_link` 대신). Arducam 왜곡 보정 값은 `camera_info`에서 받는다. tag_pose는 광학 프레임 그대로 낸다. 새 영상이 없으면 결과를 다시 내지 않는다 | 시뮬레이션(Isaac Sim 토픽)과 실물이 같은 코드로 돌게 하려고. SDK로 열면 cuVSLAM·Nav2 등 다른 노드가 같은 카메라를 쓸 수 없다. 우리 URDF의 카메라 링크 원점은 스테레오 중점이라, 기존 방식은 컬러 렌즈와 약 40 mm 어긋난다 |
| `include/perception/geometry.hpp` | `optical_to_link` 삭제, `depth_scale_for_encoding` 추가, `median_depth`가 CV_32F를 받음 | 위와 같다 |
| `src/tfmini_node.cpp` | 토픽 `/arm_range`(팔 끝) → 설정값 `range_topic`(기본 `/tfmini/range`). 새 측정이 있을 때만 발행하고, stamp는 프레임을 받은 시각. 끊기면 inf를 계속 낸다 | TFmini는 팔 끝이 아니라 1층 정면 바닥에 단다(사용자). 같은 측정을 두 번 내지 않게 하려고 |
| `CMakeLists.txt`, `package.xml` | librealsense2 의존 제거, cv_bridge·std_msgs 추가, 라이다 노드 라이브러리 추가 | 위와 같다 |
| `config/perception.yaml` | 카메라 토픽·frame 설정으로 다시 썼다. 검출·추적·태그 값은 그대로 | 위와 같다 |
| `config/arm_cam_calib.yaml` | 삭제 | 보정값은 Arducam 드라이버의 camera_info로 받는다 |
| `config/tfmini.yaml` | `range_topic` 추가, 설명 고침 | 위와 같다 |
| `test/test_vision.cpp` | `optical_to_link` 시험을 광학 좌표·깊이 형식 시험으로 바꿨다 | 함수가 바뀌어서 |
| `launch/perception.launch.py` | 다시 썼다: 인지·라이다 노드를 한 컨테이너에 넣고, 인자는 `use_sim_time`, `use_tfmini`, `config_dir` | 위와 같다 |

## 3. 새로 쓴 것

- `include/perception/frame_gate.hpp`, `scan_merge.hpp`, `obstacle_stop.hpp`(ROS 의존 없는 계산부)
- `src/scan_merger_node.cpp`(라이다 목록 합치기, roll π 장착은 TF로 처리, 차체 상자 거르기)
- `src/obstacle_stop_node.cpp`(TFmini 비상 정지 판정)
- `config/lidar.yaml`
- `test/test_inputs.cpp`, `test/test_lidar.cpp`

## 4. 추가 (2026-10-22 모의 시험 뒤)

- `include/perception/tf_helper.hpp`(새로 씀): 모의 시험 M3·M4에서 시계 되감기 뒤 고정 TF가 사라지는 결함을 찾아 고쳤다. 고정 변환을 기억하고, 되감기 때 TF 리스너를 새로 만든다. perception_node와 scan_merger가 쓴다.
- `scripts/mock_check.py`, `scripts/run_mock_check.sh`(새로 씀): 모의 시험 M1~M5. URDF를 파라미터 파일로 넘기는 방식은 이번에 겪은 인자 길이 한도(143 KB) 때문이다. 백그라운드 작업을 `set -m`으로 그룹째 끄는 방식은 이전 모듈 HANDOFF_module5의 교훈("스크립트에서 &로 띄운 프로세스는 SIGINT 무시 → set -m")을 따랐다.
