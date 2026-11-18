# YOLO 후보 B 고정 통보 · 2026-11-01 00:14

사용자 결정(2026-11-01): 후보 B 를 지금 고정하고, 가중치가 바뀌면 다시 통보한다. 받는 쪽 = '인지 코드 1101 1' 세션(최종 인지 연결 검증).
연결 검증은 인터페이스(클래스 순서 · 입출력 형상 · 후처리 · 토픽 · QoS · 엔진 형식)를 보는 것이라, 같은 4클래스 · 같은 입력으로 다시 학습하면 가중치 SHA 만 바뀐다.

## 파일과 SHA256
| 파일 | sha256 |
|---|---|
| `261030_1325/prototype/yolo_models/yolo11n_task4_B.pt` | e754b74318c8c7acb8a66e909f6fbbc2832c94445688f6a39fbec38c2d095587 |
| `261030_1325/prototype/yolo_models/yolo11n_task4_B.onnx` | d9214dafc3ef2eadd747f3f1e83154d86559e452273cae4f025015c1b3f09350 |
| `261030_1325/prototype/yolo_models/classes.txt` | eb22ffb1840f10ae1ed7262aba32dc2e9eabc3c5a9e6dd4f25b7fd81b20abca8 |
| `261030_1325/prototype/yolo_task4.yaml` | 300173b3f7330a1c3af4d813c6eced724a58ce5dfa794c934f2dc650904d3aa0 |
| `261030_1325/prototype/perception/src/trt_detector.cpp` | b0300f626d71025c7b0604ab6e249398023762f6d96aa54ee9c985458386bc10 |
| `261030_1325/prototype/perception/include/perception/yolo_postprocess.hpp`(새 파일) | 86a0fddf208f9c0f04ffabcdfcbc5726c81af8e89d002bad1ace0ee52c8ae8a0 |
| `postprocess.patch`(이 폴더, origin/dev d04599b `workspace/ros2_ws/src/perception` 기준 diff) | d776b354477049938be9631afed9e1405f7ee310fd24d1f0945ac89d89f4626e |
| PC 엔진 `261031_0100_trt/results/yolo11n_task4_B_fp32_notf32.engine`(PC 전용, Jetson 에서 쓰지 않음) | 23a5100e4dd65ced6832d438270407ad7f820c2b1dd869331a9fb9ce48a35d02 |

## 인터페이스
- 클래스(id → 색 · 모양): 0 red_cylinder(빨강 · 원기둥), 1 green_cylinder(초록 · 원기둥), 2 red_cuboid(빨강 · 직육면체), 3 green_cuboid(초록 · 직육면체). 파랑 없음.
- ONNX: 정적 입력 [1,3,640,640](RGB, /255, 정사각 letterbox 114) → 출력 [1,8,8400](cx, cy, w, h + 4클래스 점수), FP32, opset 17, NMS 없음.
- 후처리: 실수 좌표 · 클래스별 NMS, 최대 300, score 0.25 · nms 0.45(`yolo_task4.yaml`).
- 엔진 형식: **FP32(`trtexec --onnx=<onnx> --noTF32 --saveEngine=<engine>`)**. PC TensorRT 10.16.1 에서 ONNX Runtime 과 203/203 같음(상자 차 최대 0.00055 px, 점수 차 2.5e-6). FP16 은 시험 안 함. Jetson 엔진은 Jetson 에서 다시 빌드하고 같은 방식으로 다시 확인해야 함(미검증).
- 노드 파라미터(팀 perception_node, origin/dev): `detector.engine_path`(비우면 색 검출기), `detector.class_names` · `detector.input_size` 는 dev 노드에 선언 없음 → 인지 쪽 fix/perception-yolo-class-map 가 다룸.
- 토픽 · QoS(dev perception_node 기본값, 이 후보가 바꾸지 않음): 앞 `/top_camera_front/color/image_raw` · `/aligned_depth_to_color/image_raw` · `/color/camera_info`, 뒤 `/top_camera_rear/...` 같은 꼴, 팔 `/arm_camera/image_raw` · `/arm_camera/camera_info`. `image_qos` 기본 sensor_data(best effort) · `image_qos_depth` 2.

## 한계(고정 통보에 함께 보냄)
- 실물 평가 203장: TP 366 · FP 9(C++ 후처리), 초록 원기둥 65/77 = 84.4 %(누락 12).
- 학습 · 평가 자료에 팔 카메라 영상 0장 → 팔 영상(ARM_ALIGN)에 YOLO 를 쓰는 경로는 미검증.
- 어둡고 먼(1.2 m 넘게) 사진 111장(라벨 확정 470상자)은 학습에 넣지 않음. 초안 기준 모양 혼동 69 · 누락 126(도구 확인, 판정 아님).

## 받은 쪽 확인(인지 코드 1101 1, 2026-11-01)
- `postprocess.patch` 는 인지 PR 9개를 합친 트리(756c2b4)에서 **컴파일 안 됨**: 시제품 trt_detector.cpp 가 10-29 시제품(261029_1408/yolo_classmap.patch)의 `perception/puck_classes.hpp` · `valid_puck_classes()` · `DetectorConfig::class_names` 를 씀. 팀 PR #22 는 다른 설계(`puck_class.hpp`, `class_names_mismatch()`, `Detector::num_classes()`, 노드에서 검사).
- 처리: #22 설계에 맞춘 이식은 인지 코드 쪽이 함(사용자 결정 대기). 옮길 후처리 동작 = 실수 좌표 · 클래스별 NMS · 최대 300 · score 0.25 · nms 0.45. dev `detector.score_threshold` 0.3 은 색 검출기와 같이 쓰는 값. 기록 `261101_0014_merge9/README.md`.
- 이식 시작(인지 코드 1101 1, 사용자 결정 2026-11-01): 브랜치 fix/perception-yolo-postprocess(#22 fa86c15 위), 작업 트리 `~/git/Lv2_Cartlidar_Assignment_yolopost`. decode_task_yolo 동작 그대로 · 입출력 검사 옮김 · 클래스 수는 #22 num_classes/class_names_mismatch · 엔진 전용 `detector.engine_score_threshold`(기본 0.25) 추가 예정. 확인 = 단위 시험 + PC 엔진으로 팀 TensorRTDetector 203장 vs 261031_0100_trt(클래스 · 개수 203/203, 상자 ≤ 0.5 px, 점수 ≤ 1e-3).
- 이쪽 약속: 이식 중에는 postprocess.patch · 시제품 두 파일을 바꾸지 않음(바꾸면 그쪽에 알림).
