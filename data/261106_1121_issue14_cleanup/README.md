# #14 정렬 기준점 · 인지 코드 정리 · 2026-11-06 (모의 · 정적 확인, 실물 아님)

## #14 정렬 기준점 (`fix/perception-arm-align-point`, 작업 트리 `~/git/Lv2_Cartlidar_Assignment_alignpt`, dev 30c2d1f 위)
- `arm.align_point` [폭 비, 높이 비]: `arm_offset` x · y 의 0 이 되는 화면 자리. 기본 [0.5, 0.5] 이라 지금 값과 같다. 범위 밖 값은 기동 때 거부한다.
- 잠금 · 평면 교차가 쓰는 내부 좌표는 화면 중심 기준 그대로 두고, 내보내는 값만 바꾼다.
- 왜 기본을 (320, 400) 으로 안 바꿨나: 기준 모델 B안(잡는 점 gripper_roll_link x 0.14 m, 카메라 15° 기울임, f 283 px 가정 · 640×480)에서 집게 축 위 점의 세로 위치는 거리마다 다르고, 잡는 점 자체는 화면 밖이다.

| 잡는 점에서 앞으로 | 0 | 2 cm | 4 cm | 6 cm | 11 cm | 16 cm | 26 cm |
|---|---|---|---|---|---|---|---|
| 세로 v [px] | 552 | 456 | 398 | 359 | 302 | 271 | 237 |

- 쓰는 곳: drive FINE 은 x 만(가로 기준은 화면 중심 그대로). y 는 manipulator 옛 pick 보정만(`toppick.use_alignment` false 라 toppick_tilt 는 안 씀).
- 시험: colcon test 214 · 실패 0(새 Geometry.AlignOffset 포함), 기동 FATAL 0, 범위 밖 값 [0.5, 1.5] 는 생성자 예외로 안 뜸. 기본값이 산술로 같아 A1 · F1 은 다시 안 돌렸다.
- 한계: 실제 값은 실물 해상도(#119) · 보정 뒤 정렬 거리에서 재야 한다. PerceptionResult.msg 주석 '화면 중심 대비' 는 기본값에서 맞고, 메시지는 인지 밖이라 안 고쳤다.

## 코드 정리 (`fix/perception-cleanup`, 작업 트리 `~/git/Lv2_Cartlidar_Assignment_cleanup`, dev 30c2d1f 위)
조사: cppcheck(unusedFunction · warning · style) · pyflakes · 빌드 경고 · yaml ↔ declare_parameter 대조.
- 고침: 안 쓰는 import(barcode_compare.py `os`), test_arm_plane 복사 경고 1건, test_perception_logic TIMEOUT 300(기본 60 s 인데 PC 부하 때 53 s 이상 걸려 시간 초과로 실패함).
- 지움: `scripts/mock_check.py` · `mock_check_pucks.py` · `run_mock_check.sh`. 10월 초 시험이고 지금 코드에서 동작하지 않는다.
  - mock_check.py: B안 URDF 에서 KeyError lidar_front_scan 으로 멈춤, M1 은 파란 사각형(파랑은 끔).
  - mock_check_pucks.py: P1~P5 모두 실패, READ_TAG 메시지 0(원인은 안 봄, 부하 높을 때 돌림).
  - 대신 쓰는 시험: 렌더 재생 A1 · F1 · R 시리즈(`Cartlidar/sim_render`, 저장소 밖).
- 안 고침(오탐 · 취향 · 진행 중 브랜치와 충돌): cppcheck arrayIndexThenCheck 2건은 오탐(순서 맞음), trt_detector log 는 가상 함수, useStlAlgorithm 27 · shadow 6 · const 참조 등은 동작 차이 없어 둠. perception_node.cpp 2056줄 쪼개기는 커밋 전 브랜치(바코드 · 태그)와 #120 이 끝난 뒤가 낫다.
- 시험: colcon test 213 · 실패 0 (gtest 183 · pytest 24 포함). 제품 코드는 안 바뀌었다.
