# 팔 전용 엔진 arm.engine_path + D435 480 · 팔 384 FP16 엔진 · 2026-11-04 (모의, PC)

무엇: YOLO 1102 2 가 11-04 에 통보했다(근거 `261104_1005_yolo_D_480/README.md`, `261104_0941_fp16_check/README.md`).
- 가중치는 D 그대로이고 엔진만 바뀐다.
- D435 는 1×3×480×640 FP16 엔진을 쓴다. 정사각 640 은 640×480 장에 위아래 80 px 띠를 붙여 재현율이 낮았다.
- 팔(Arducam)은 1×3×384×640 FP16 엔진을 쓴다. 480 엔진을 팔에 쓰면 정밀도가 0.943 으로 불통과다.
- 지금 인지에는 팔 엔진 경로가 따로 없다(arm.detector main = 주 검출기 인스턴스를 같이 씀). 그래서 `arm.engine_path` 를 넣는다.

코드: 브랜치 `fix/perception-arm-engine-path`(작업 트리 `~/git/Lv2_Cartlidar_Assignment_armengine`, `fix/perception-arm-lock-yolo` 4738939 = PR #96 위).
- `perception_node.cpp`: `arm.engine_path` 를 받는다.
  - 비우면 지금과 같다.
  - 주어지면 팔만 그 엔진을 따로 연다. 클래스 수가 주 검출기와 다르면 FATAL 이고, 못 열면 WARN 을 남기고 주 검출기를 쓴다.
  - 주 검출기가 엔진이 아니거나 arm.detector 가 color 면 무시하고 WARN 을 남긴다.
  - 팔 엔진은 번호 뜻이 같으므로 target_class 도 쓴다.
- `perception.yaml`: `detector.engine_path` 를 Jetson 480 FP16 으로, `arm.engine_path` 를 Jetson 384 FP16 으로 바꾼다.
- 도구 `Cartlidar/sim_render/set_yaml_values.py`: 같은 이름 절이 여럿이면(cameras.arm · arm) 이름이 있는 첫 절에서 바꾼다. 예전에 KeyError 가 나던 경우만 달라진다.

PC 엔진: 같은 ONNX(`261104_1005_yolo_D_480/pkg/*.onnx`)로 `trtexec --fp16` 해서 `trt/` 에 만들었다(TensorRT 10.16.1, rc 0 · 0). 판정은 PC 엔진으로 한다.

실행: `bash run_all.sh` (PC). 결과 `result.log`, `boot/`, `a1_480_384/`, `f1/`.

| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| 빌드 · 단위 | 모두 통과 | 통과, 203 개. 새 코드는 노드 생성자만 바뀌어 단위 시험 수는 같음 | **통과** build rc 0, 203 tests · 0 errors · 0 failures |
| 기동 both (480 + 팔 384) | '검출기: tensorrt … 입력 640×480 … · 팔 tensorrt … 입력 640×384', FATAL 0 | 그대로 나옴. PC 엔진 입출력이 float32 라 trt_detector 검사 통과 | **그대로**: '입력 640×480, 클래스 4 · 팔 tensorrt … 384 … (입력 640×384, 클래스 4)', FATAL 0 |
| 기동 shared (팔 경로 비움) | '팔 tensorrt', 팔 엔진 정보 없음 | 지금과 같음(주 검출기 같이 씀) | **그대로** '팔 tensorrt'(팔 엔진 정보 없음), WARN 없음 |
| 기동 arm_missing | WARN 'arm.engine_path 를 쓰지 못해' + '팔 tensorrt' | 그대로 | **그대로**: WARN '…쓰지 못해 팔도 주 검출기를 씀: 엔진 파일을 열 수 없음' + '팔 tensorrt' |
| 기동 no_engine (주 엔진 없음) | 주 color 대체 WARN + 'arm.engine_path … 무시' WARN, 팔 color+label | 그대로 | **그대로**: 'color(any) … ColorDetector 사용 · 팔 color+label(any)' + '… 무시' WARN |
| A1 3차 묶음(시드 20261106, 12 장면) | v12 기준(10-29): P1 100 % · P2 ≥ 95 % · 다른 퍽 0 · 가림 100 % · P4 ≥ 95 % | 통과. 640 FP32 때(P1 12/12 · P2 204/212 · 0 · 40/40 · 10/10)와 ± 몇 장. 팔 입력이 640×640 → 640×384 라 같은 640×360 영상의 배율은 같고 띠만 줄어 박스는 거의 같음 | **통과** P1 12/12 · P2 204/212 = 96.2 % · 다른 퍽 0 · 가림 40/40 · 복귀 10/10. 실패 장 8개와 계획 · 선택 불일치 2개(w5)는 640 FP32 때와 같은 장이고, w3 오차만 0.0605~0.0621 → 0.0590~0.0611 로 조금 다름 |
| F1 24회(B0′) | 24/24 · 288/288 · 틀린 완료 0 · ≤ 10 mm | 통과. 최종 중심은 팔 영상 측량이라 640 FP32 때(최대 5.33 mm) 근처 ± 1 mm. D435 퍽 수는 띠가 없어져 같거나 많음 | **통과** 24/24 · 288/288 · 틀린 완료 0 · 최대 5.10 mm · 누운 칸 80/80 · 무응답 0 · 기동 12/12 (480 + 팔 384). 예상과 다른 점: D435 퍽은 642 → 613 개(−4.5 %)로 줄었다(퍽 있는 장 282 같음, x < 0.35 헛 퍽 0 · 0). 렌더 모의 영상 기준이고, 실물 사진 재현율은 YOLO 쪽 판정에서 올랐다 |

결과(2026-11-04 11시, PC 엔진 · 모의): **빌드 · 단위 · 기동 4가지 · A1 · F1 모두 통과**. `result.log`, `boot/*.log`, `a1_480_384_score.log`, `f1/replay_*/summary.json`.
- D435 퍽 수 비교: 같은 F1 렌더에서 `261104_0056_tf_wait/f1`(640 FP32)의 D435 퍽 642 와 이번 613 을 비교했다. 판정 항목은 아니다.

## 한계
- Jetson 엔진(TensorRT 10.16.2)으로 노드를 띄우지 않았다. 입출력 float32 · 입력 모양은 PC 엔진으로만 확인했다(같은 ONNX · 같은 --fp16).
- 렌더 영상에서는 480 엔진의 D435 퍽 수가 4.5 % 적다. 실물 재현율 개선은 YOLO 쪽 판정(1478장)이다.
- 두 엔진을 같이 열어 GPU 메모리가 늘어난다(엔진 파일 각 약 7 MB). Jetson 부하는 YOLO 쪽이 FP16 판으로 다시 잴 예정이다.

## Jetson 기동 확인 (도구 확인, 사용자 실행) — `jetson/check_boot.sh` + `jetson/src_02bd141.tar.gz`
| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| 엔진 sha256 | 앞자리 480 58df0823 · 384 e22717e8(YOLO 통보) | 같음 | **같음** 58df0823c38c36af… · e22717e897fc60e4… |
| 빌드 | build_rc 0, 'perception: TensorRT 사용' | 그대로(Jetson 에 TensorRT · CUDA 있음) | **통과**(세 번째에): 'perception: TensorRT 사용 (/usr/lib/aarch64-linux-gnu/libnvinfer.so)'. 처음 두 번은 Jetson 에 libzbar-dev, liblapack-dev · libblas-dev 가 없어 실패 → 사용자가 apt 로 설치(libopencv-contrib-dev 는 이미 있음) |
| 기동 | '검출기: tensorrt … 480_fp16 (입력 640×480, 클래스 4) · 팔 tensorrt … 384_fp16 (입력 640×384, 클래스 4)', FATAL 0, 엔진 WARN 없음 | 그대로. 같은 ONNX · --fp16 이라 입출력 float32 | **그대로**: '… 480_fp16.engine (입력 640×480, 클래스 4) · 팔 tensorrt … 384_fp16.engine (입력 640×384, 클래스 4) · 태그 사용 가능 · 마커 사용 가능', FATAL 0, 엔진 WARN 없음 |

결과(2026-11-04 오후, Jetson, 사용자 실행 · 터미널 출력 확인): **Jetson 기동 확인 통과**. C++ 인지 노드가 Jetson 에서 처음 빌드 · 기동됨. 카메라 없이 띄워 검출 · 부하는 보지 않음.
