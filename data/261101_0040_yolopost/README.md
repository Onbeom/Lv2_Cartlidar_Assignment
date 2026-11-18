# YOLO 후보 B 후처리 팀 코드 이식 (2026-11-01, 사용자 '이식 진행')

- 브랜치 `fix/perception-yolo-postprocess`(작업 트리 `~/git/Lv2_Cartlidar_Assignment_yolopost`), 시작점 #22 `fix/perception-yolo-class-map` fa86c15. PR base 는 #22 브랜치(쌓인 PR)를 권함 — #22 의 `num_classes()` · `class_names_mismatch()` 를 씀.
- 근거: YOLO 1101 1 고정 통보 `261101_0014_yolo_fix/`(postprocess.patch 는 10-29 시제품 이름을 써서 합친 트리에서 컴파일 안 됨 — `261101_0014_merge9/README.md`).
- 바뀐 것:
  - 새 `include/perception/yolo_postprocess.hpp` `decode_yolo()`: 통보의 `decode_task_yolo` 와 같은 동작(실수 좌표, 클래스별 NMS, 점수 순 최대 300, 유한 · 점수 ≤ 1 검사, 화면 밖 상자 버림). 이름 · 꼴만 팀 코드에 맞춤.
  - `trt_detector.cpp`: 위 함수로 후처리, 입력 1개 · 출력 1개 · 정적 float32 1×3×H×W / 1×C×N 검사 추가, 엔진 만들기 예를 FP32 `--noTF32` 로. 클래스 수 검사는 노드의 #22 방식 그대로(통보 패치의 `valid_puck_classes` · `DetectorConfig::class_names` 는 안 씀).
  - 새 파라미터 `detector.engine_score_threshold`(기본 0.25, 엔진만). `detector.score_threshold` 0.3 은 색 검출 점수(채움 · 가로세로비)라 뜻이 달라 그대로 둠 → 색 검출 동작은 바뀌지 않음.
  - 시험 `test/test_yolo_postprocess.cpp` 5개(클래스별 NMS, 레터박스 실수 복원, 문턱 · target_class · 자르기, 나쁜 값, 최대 300).

| 항목 | 예상(실행 전) | 결과 |
|---|---|---|
| 브랜치 단독 빌드 · 시험 | 빌드 통과(TensorRT 경로 켜짐), 시험 = #22 단독 수 + 5, 실패 0 | 1차: 137 gtest 중 1 실패(아래). 고친 뒤 **gtest 137/137**(#22 단독 132 + 5) · 합계 138/138. test_rc 5 는 #22 시점 perception_survey 에 pytest 가 없어서(이전 build_team 과 같음) |
| 인지 PR 9개 합친 트리 + 이 변경 | 빌드 통과, 157 + 5 = 162, 실패 0 | 1차 같은 1 실패. 고친 뒤 **162/162 통과**(gtest 144 · camera_info 4 · ctest 2 · pytest 12) |
| 팀 TensorRTDetector ↔ ONNX Runtime + decode_cli(PC 엔진, 실물 203장, 이전 기준) | 클래스 · 개수 203/203, 상자 차 ≤ 0.5 px(예상 0.01 px 이하 — 같은 후처리 · 같은 엔진, 레터박스는 같은 cv::resize), 점수 차 ≤ 1e-3 | **통과**: 203/203, 검출 375 = 375, 상자 차 최대 6.1e-5 px, 점수 차 최대 2.9e-6(`equiv/summary.json` · `rows.json`). 엔진 sha256 23a5100e…, ONNX d9214daf…(통보와 같음) |

- 1차 실패(예상과 다름, 시험 쪽 가정 틀림): `ThresholdTargetAndClip` 에서 점수 = 문턱 0.25 인 상자를 남는다고 적었는데, `cv::dnn::NMSBoxes` 는 문턱보다 **큰** 점수만 남겨 버려짐. 통보 받은 `decode_task_yolo` 도 같은 동작이라 코드는 그대로 두고 시험을 '0.25 버림 · 0.26 남김'으로 고침. 1차 결과 `build_*/test_result_try1.txt`.
- 합친 트리 + 이 변경: 작업 트리 `~/git/Lv2_Cartlidar_Assignment_merge9yp`(756c2b4 + `yolopost.patch`, 미커밋). CMakeLists 는 #23 의 tf_lookup 줄 때문에 문맥이 달라 `git apply -3` 로 붙음(충돌 없음).
- 동등성 도구: `equiv/trt_team.cpp`(팀 `make_detector(type tensorrt)`, 브랜치 빌드의 libperception_detector.a), `equiv/compare.py`(ORT CPU 최적화 끔 + `261030_1325/yolo_cpp/decode_cli`), `equiv/run_equiv.sh`(GPU 잠금). PC 전용 — Jetson 아님.
