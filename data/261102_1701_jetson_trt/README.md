# Jetson TensorRT FP32 엔진 동등성(2026-11-02 17:01, 실물 보드 · 주행 없음)
왜 하는가: 새 교안(63–69강)에 따르면 TensorRT 엔진은 쓸 보드에서 만들어야 한다. 그런데 지금은 PC 엔진 동등성만 확인했다.

방법:
- `make_package.py`(PC)로 `pkg/`를 만든다.
  - 입력: 실물 평가 203장. PC에서 letterbox한 640² PNG로 저장해, 양쪽 cv2 판 차이를 없앴다.
  - 모델별 ONNX와 ONNX Runtime CPU 기준 출력.
- `jetson_check.py`(Jetson)가 하는 일:
  - 보드 정보를 기록한다.
  - `--noTF32`가 있는지 확인한다.
  - 엔진을 빌드하고 203장을 비교한다.
- 후처리 decode(conf 0.25 · 클래스별 NMS 0.45)는 PC에서 팀 C++ 디코더(decode_cli)와 203/203장 같게 나오는 것을 확인했다(도구 확인).
- 판정 기준은 B · C의 PC 확인 때와 같다: 모든 장에서 검출 수 · 클래스가 같고, 상자 차 ≤ 0.5 px, 점수 차 ≤ 1e-3.

| 항목 | 예상(실행 전) |
|---|---|
| Jetson trtexec `--noTF32` | 있음(JetPack의 TensorRT 10.x로 예상). 없으면 멈추고 board.json으로 판을 확인한다 |
| B · C 203장 동등성 | 통과(PC 0.00052 px 수준). Orin GPU의 tactic이 달라 상자 차가 PC보다 조금 클 수 있으나 0.01 px 이하 |
| 시간 | 모델당 빌드 약 3~5분 + 203장 × trtexec 실행 약 1초 ≈ 모델당 8분 |

## 결과 — D(11-03 15:3x, 사용자가 Jetson에서 실행, 터미널에서 읽음)
- 보드: aarch64, Ubuntu 24.04.5, L4T R39.2.1, 전력 모드 25W, TensorRT 10.16.2.10(trtexec v101602), cv2 4.6.0, `--noTF32` 있음.
- D: 203/203장 같음. 검출 376개(conf 0.25 기준), 상자 차 최대 0.00052 px, 점수 차 2.0e-6, raw 최대 0.0074 → **통과**(기준 ≤ 0.5 px · ≤ 1e-3).
  - 엔진 sha256 dbbf4a0babfc569a9acd493eecee908ff303c54a2c7b1ac560b4cb81fb1a181a(Jetson `~/trt_check/jetson_results/yolo11n_task4_D_fp32_notf32.engine`).
- 예상(PC 수준, 0.01 px 이하)대로였다. B · C는 Jetson에서 돌리지 않았다(D만 실행).
