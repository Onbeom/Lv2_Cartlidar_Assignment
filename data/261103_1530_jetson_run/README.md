# Jetson에서 D 돌리기(2026-11-03 15:30, 실물 보드 · 주행 없음)
Jetson 확인 결과(사용자 실행 15:2x):
- 있음: tensorrt 10.16.2.10, cv2 4.6.0, trtexec.
- 없음: torch, ultralytics, onnxruntime, pycuda, cuda-python.
- 그래서 `.pt`는 그대로 못 돌린다. D ONNX → 이 보드 TensorRT 엔진(jetson_check.py가 만듦) → `trt_run.py`로 돌린다.
  - trt_run.py는 tensorrt와 libcudart(ctypes)와 OpenCV만 쓴다.
- D `.pt`는 `~/yolo_D/`로 옮겼다(sha256 35f7af1a… PC와 같음).

`trt_run.py`:
- 전처리 · 후처리는 학습 · 평가와 같다: letterbox 114, conf 0.25, 클래스별 NMS 0.45, 표시 문턱 0.6.
- 입력은 세 가지 중 하나다: 이미지 폴더, V4L2 장치(Arducam), `rs:<시리얼>`(D435 컬러, pyrealsense2가 있는 .venv 파이썬으로).
- **PC에서는 돌려 보지 않았다.** GPU를 Nav2 세션이 쓰고 있었다.

Jetson 자체 확인: `~/trt_check/inputs`(203장, PC에서 letterbox한 입력)에서 0.6 이상 검출 수가 PC ONNX Runtime 기준과 같은지 본다.
- 기준은 `ref_D_show0.6.txt`이고, 0.6 이상 검출은 370개다.

| 항목(도구 확인) | 예상(실행 전) |
|---|---|
| 엔진 로드 · 실행 | 된다(TensorRT 10.16 API, 정적 입출력) |
| 203장 0.6 이상 검출 수 | 370(기준과 같음). 점수가 0.6 경계에 걸린 것이 있으면 ±1 |
| 전처리+추론+후처리 지연 p50 | 15~30 ms(Orin Nano, yolo11n FP32). 창 표시(ssh -X)는 따로 더 느림 |
