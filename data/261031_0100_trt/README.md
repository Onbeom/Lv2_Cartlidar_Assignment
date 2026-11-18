# 후보 B · PC TensorRT FP32(--noTF32) 동등성 · 2026-10-31 01:xx (모의 아님, PC 추론 도구 확인)

사용자 묶음 승인(2026-10-31 00:5x). 교안 Y1(ONNX → TensorRT --noTF32, ONNX Runtime 출력과 수치 비교)을 후보 B 에 적용. **Jetson 동등성 아님**(엔진은 대상 GPU 에서 다시 만들어야 함).
- 모델 `…/improvement_261030_1325/yolo_export/yolo11n_task4_B.onnx`(sha256 d9214dafc3ef2ead…, 정적 [1,3,640,640] → [1,8,8400], FP32, NMS 없음).
- 입력 실물 평가 203장(`261030_1325/B_negative/images/eval`), 전처리는 evaluate_cpp_yolo.py 와 같음(정사각 letterbox 114, RGB, /255).
- 기준값 ONNX Runtime CPU(최적화 끔, 이전 동등성 통과 설정). 후처리는 같은 C++ `decode_cli`(클래스별 NMS).
- 엔진: TensorRT 10.16.1(PC, RTX 5060 Laptop) `trtexec --onnx --noTF32 --saveEngine`(FP16 끔). 장마다 `trtexec --loadEngine --loadInputs --exportOutput`.

| 항목 | 기준(사용자 승인) | 예상(실행 전) |
|---|---|---|
| 장마다 검출 클래스 · 개수 | 203/203 같음 | 203/203 같음(신뢰도 문턱 근처 상자가 있으면 1~2장 다를 수 있음) |
| 짝 지은 상자 좌표 차 | ≤ 0.5 px | 최대 0.01 px 이하(교안 max abs 0.000954 수준) |
| 짝 지은 점수 차 | ≤ 1e-3 | 1e-5 안팎 |
| 원시 출력 [1,8,8400] 최대 차(참고) | — | 1e-3 이하 |

## 결과(01:2x, `results/summary.json` · `results/rows.json`, 엔진 `results/yolo11n_task4_B_fp32_notf32.engine`, 빌드 기록 `results/trtexec_build.log`)
| 항목 | 기준 | 결과 | 예상과 비교 |
|---|---|---|---|
| 장마다 검출 클래스 · 개수 | 203/203 | **203/203 같음**(검출 375개 = 이전 C++ helper TP 366 + FP 9 와 같은 수) | 예상대로 |
| 짝 지은 상자 좌표 차 최대 | ≤ 0.5 px | **0.00055 px** | 예상(≤ 0.01) 안 |
| 짝 지은 점수 차 최대 | ≤ 1e-3 | **2.5e-6** | 예상(1e-5 안팎)보다 작음 |
| 원시 출력 최대 차(참고) | — | 0.0062(상자 좌표 칸, 640 px 단위) | 예상(1e-3)보다 큼 — 좌표가 px 단위라 상대 오차는 1e-5 수준, 후처리 결과에는 영향 없음 |
- **판정: 통과(PC TensorRT 10.16.1 FP32 --noTF32, RTX 5060 Laptop).** Jetson 엔진은 대상 GPU 에서 다시 빌드 · 같은 방식으로 다시 확인해야 함(미검증). FP16 엔진은 시험하지 않음.
