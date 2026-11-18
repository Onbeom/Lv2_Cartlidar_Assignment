# 인지 PR 9개 합친 트리 빌드 · 시험 (2026-11-01, 도구 확인 · PC)

- 소스: origin/dev d04599b 위에 #21 847da66 → #22 fa86c15 → #23 a965442 → #24 e23f848 → #25 2ca1d92 → #26 5d0c363 → #27 833242a → #28 e1c4cad → #29 f0b1b18 차례로 합침(git merge-tree, 충돌 0) = 임시 커밋 756c2b4(브랜치 없음).
- 작업 트리: `~/git/Lv2_Cartlidar_Assignment_merge9`(detached 756c2b4). 빌드 `build/`(이 폴더).
- 판정 소스(`_judge261031` 91e774c + 미커밋)와 비교: perception · perception_survey 코드 · 시험 파일 같음. 다른 것은 `perception.yaml` 하나뿐 — 합친 트리는 #28 의 D3 값이 기본값, 판정 트리는 이전 기본값 + 판정 때 REPLAY_SET 으로 D3 값을 줌(값은 같음, 주석만 다름).
- 명령(이전 build_team 과 같은 방식): `colcon build --base-paths <작업 트리>/workspace/ros2_ws/src --packages-up-to perception perception_survey` → `colcon test --packages-select perception perception_survey` → `colcon test-result --all`.

| 항목 | 예상 | 결과 |
|---|---|---|
| 빌드 | 3개 패키지(robot_interfaces · perception · perception_survey) 성공. 코드가 판정 빌드 build_s1 과 같으므로 | 통과(build_rc 0, 3개 패키지, TensorRT 경로 USE_TENSORRT 켜짐 — PC libnvinfer 10.16.1) |
| 시험 수 · 실패 | 157개 · 실패 0(gtest + pytest). 판정 빌드 build_s1 157/157 과 시험 파일이 같으므로 | **157/157 통과**(gtest test_perception_logic 139 · test_camera_info 4 · ctest 2, pytest perception_survey 12), 실패 0 · 건너뜀 0 |

로그: `build/build.log` · `build/test.log` · `build/test_result.txt` · `build/result.log`.

## YOLO 후보 B 후처리 패치를 합친 트리에 붙이기 (도구 확인, 2026-11-01)

- 통보 받은 `261101_0014_yolo_fix/postprocess.patch`(d04599b 기준, trt_detector.cpp · 새 yolo_postprocess.hpp)는 합친 트리에 글자로는 붙지만(`git apply --check` 통과) **컴파일 안 됨**(scratchpad 사본, perception_detector 대상만 빌드):
  1. `#include "perception/puck_classes.hpp"` · `valid_puck_classes()` — dev 에도 합친 트리에도 없음. 10-29 시제품(`261029_1408/yolo_classmap.patch`)의 이름. 팀 PR #22 는 다른 설계(`puck_class.hpp` · `class_names_mismatch()` · `Detector::num_classes()`, 노드에서 검사)로 들어감.
  2. 위 두 줄을 빼도 `DetectorConfig::class_names` 없음(trt_detector.cpp 73행) — #22 는 class_names 를 노드(`class_names_`)에 두고 검출기 설정에는 넣지 않음.
- 그 밖에 맞출 것: 합친 트리 `detector.score_threshold` 0.3 은 색 검출기와 같이 씀(YOLO 후보 0.25), `detector.input_size` 는 노드 파라미터 선언 없음(DetectorConfig 기본 640 = 후보 입력과 같음).
- 결론: YOLO 후처리는 #22 설계에 맞게 이식해야 최종 연결 검증을 할 수 있음(클래스 수 검사는 #22 의 num_classes · class_names_mismatch 를 쓰고, trt_detector 에서는 출력 채널 ≥ 5 · 입출력 형식만 확인).
