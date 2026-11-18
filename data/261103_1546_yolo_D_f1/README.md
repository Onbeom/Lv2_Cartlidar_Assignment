# YOLO 후보 D 로 F0 · F1 다시 · 2026-11-03 (모의, PC)

무엇: YOLO 1102 2 가 11-03에 통보했다. 사용자가 후보 D를 '임시 고정'했고 Jetson 동등성도 통과했다.
- ONNX: `261102_1634_yolo_D/yolo_export/yolo11n_task4_D.onnx`(sha256 eeb0d6bc…, 이 PC에서 직접 확인).
- 입출력 · 클래스 순서 · 문턱은 B · C와 같다(`input_contract_D.json`).
- PC 엔진: 이 폴더 `trt/`에서 trtexec FP32 --noTF32로 만든다. C와 같은 방식이다.
- Jetson 엔진: `/home/cart11/yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine`(sha256 dbbf4a0b…, Jetson 배포 폴더)이다. 판정은 PC 엔진으로 한다.

코드: 팀 origin/dev c27d100을 새 작업 트리 `~/git/Lv2_Cartlidar_Assignment_devD`(detached)로 받아 이 폴더 `build/`에서 빌드한다.
렌더 · REPLAY_SET · URDF · mission_map 값은 C 판정(`261102_1308_yolo_C_f1/run_all.sh`)과 같고, 엔진만 D로 바꾼다. 기준도 B0′ · F1 · C 판정과 같다(사용자 11-01 확인).

| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| 빌드 · 단위 시험 | 모두 통과 | 모두 통과. dev c27d100 그대로라 바뀌는 코드 없음 | **통과**: survey 시험 rc 0, 인지 155 · 3 · 7 · 4개 통과 |
| F0 잡음(보임 ≥ 0.5) | 기준 없음(도구 확인) | 483~484/484. D는 C 자료에 실물을 더한 것이라 C(484) 근처 | **484/484** |
| F0 클래스 틀림 | 기준 없음 | 도구 셈 결함 4(seed 102 이웃 상자) 말고 0~2. C는 진짜 틀림 1 | **도구 셈 결함 4 말고 0**(클래스 맞음 480/484; 4개는 모두 seed 102, C와 같은 이웃 상자). C에 있던 진짜 모양 틀림 1이 사라짐 |
| F0 정답 중심 밖 상자 | 기준 없음 | C(185) 근처 ± 50. 렌더는 학습 자료와 영역이 달라 방향을 모름 | **223**(C 185보다 38 많음, 예상 범위 안). 같은 퍽 겹친 상자 7(C 25) |
| F1 기동 | 12/12 'tensorrt … 클래스 4' · FATAL 0 | 12/12. 입출력 모양이 같음 | **12/12** 'tensorrt … 클래스 4', FATAL 0 |
| F1 24회 · 칸 | 24/24 · 288/288 | 24/24 · 288/288. 모양은 팔 태그 표 우선(D1, dev 반영)이라 D435 한 장 틀림으로는 거부되지 않음 | **24/24 · 288/288** |
| 틀린 완료 · 틀림/모름 | 0 · 0 | 0 · 0 | **0 · 0**(ID · 색 · 모양 · 자세 모두 0) |
| 중심 오차 최대 | ≤ 10 mm | 6.07 mm 근처. 최종 중심은 팔 영상 측량이라 D435 검출기와 무관 | **5.33 mm**. 예상(6.07 근처)보다 0.74 mm 작음, 기준 안. 재생마다 관측 시점이 조금씩 달라 C와 다른 값 |
| 무응답 · 집게 헛 퍽(x < 0.35) | 0 · 0 | 0 · 0 | **0 · 0**(D435 퍽 642, C 597) |

결과 `f0/summary.json`, `f1/replay_*/summary.json`(2026-11-03 16시). PC 엔진은 TensorRT 10.16.1(Jetson은 10.16.2). **F0 도구 확인 · F1 통과(모의, PC 엔진).**

## 팀 반영 (브랜치 `fix/perception-yolo-d`, 작업 트리 `~/git/Lv2_Cartlidar_Assignment_yoloD`, dev c27d100 위)
- perception.yaml:
  - `detector.engine_path`를 Jetson 배포 경로 `/home/cart11/yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine`(sha256 dbbf4a0b…)으로 바꿨다.
  - `class_names` 4개를 켰다.
- perception_node.cpp: engine_path를 줬는데 엔진을 못 쓰고 색 검출기로 대신하면 WARN을 남긴다.
- 확인: 빌드와 인지 시험 155 · 3 · 7 · 4가 통과했다. PC 기본 설정 기동(`branch/start_default.log`)은 엔진 파일이 없어 WARN을 남기고 color로 정상 기동했다. class_names는 무시 WARN이 뜨고 FATAL은 없다.
- `arm.detector`는 color 그대로 둔다. main으로 바꾸면 A1(팔 정렬) 재판정이 필요해 사용자 결정 사항이다.

## 한계
- Jetson 엔진으로 F1을 돌리지는 않았다. Jetson 엔진과 PC ONNX의 동등성(203/203)은 YOLO 쪽 결과다.
- 렌더는 모의 자료다. 실물 커튼 조건의 D435 재현율 0.68 · 0.62는 YOLO 쪽에서 판정했고, 아직 95 %에 못 미친다(노출 가설).
