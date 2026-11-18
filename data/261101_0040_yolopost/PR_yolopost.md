제목: perception: YOLO 4클래스 엔진 후처리(클래스별 NMS · 실수 좌표)와 입출력 검사

base: fix/perception-yolo-class-map (#22 위에 쌓은 PR)

## 무엇을
- 새 `yolo_postprocess.hpp` `decode_yolo()`: 엔진 출력을 실수 좌표 그대로 원본 화면으로 되돌리고, NMS 를 클래스마다 따로 합니다(점수 순 최대 300개, 유한하지 않은 값 · 1 넘는 점수 · 화면 밖 상자 버림). `trt_detector.cpp` 는 이 함수를 쓰고, 입력 1개 · 출력 1개 · 정적 float32 모양이 아니면 시작하지 않습니다.
- 새 파라미터 `detector.engine_score_threshold`(기본 0.25, 엔진만). 기존 `detector.score_threshold` 0.3 은 색 검출 점수라 그대로 두어 색 검출 동작은 같습니다. 시험 `test_yolo_postprocess.cpp` 5개를 더했습니다.

## 왜
YOLO 후보 B(4클래스: 빨강·초록 × 원기둥·직육면체)는 클래스별 NMS 와 실수 좌표 후처리, 점수 0.25 로 실물 203장을 평가했습니다. 기존 후처리는 모든 클래스를 한꺼번에 NMS 해서 겹쳐 보이는 다른 클래스 퍽 하나를 지우고, 상자 좌표를 정수로 잘랐습니다. 클래스 수 검사는 #22 의 `num_classes()` · `class_names_mismatch()` 를 그대로 씁니다.

## 시험
- 단위 시험: 이 브랜치 gtest 137/137, 열린 인지 PR(#21~#29)과 합친 트리 + 이 변경 162/162(PC, TensorRT 경로 켜짐).
- PC 동등성(TensorRT 10.16.1 FP32 `--noTF32` 엔진, 실물 203장): 팀 `make_detector(type tensorrt)` 결과 ↔ ONNX Runtime + 평가 때 쓴 후처리 — 203/203장 클래스 · 개수 같음(검출 375개), 상자 차 최대 6.1e-5 px, 점수 차 최대 2.9e-6(기준 ≤ 0.5 px · ≤ 1e-3).

## 한계
PC 엔진으로만 확인했습니다. Jetson 에서는 엔진을 다시 만들고 같은 방식으로 다시 확인해야 합니다. FP16 엔진과 팔 카메라 영상(ARM_ALIGN)에서 엔진을 쓰는 경로는 시험하지 않았습니다.
