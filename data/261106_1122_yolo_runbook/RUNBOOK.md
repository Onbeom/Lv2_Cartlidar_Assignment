# YOLO E3 운영 문서 (RUNBOOK)

작성 2026-11-06. 대상: Jetson `cart11` 에서 도는 퍽 검출 엔진 E3. 팀 저장소 `origin/dev`(ae7c7e2) 의 perception 코드와 설정을 읽어 썼다.

**이 문서의 명령은 아직 실물에서 이 순서로 돌려 보지 않았다.** 확인한 범위는 5절을 본다.

## 1. 지금 배포된 것

| 항목 | 값 |
|---|---|
| 모델 | YOLO11n, 클래스 4개: 0 `red_cylinder` · 1 `green_cylinder` · 2 `red_cuboid` · 3 `green_cuboid` |
| 가중치 | `261104_1618_E3/runs/E3/weights/last.pt`, sha256 `4b0d809be2b6285ac41d3c3dc7cf08b68b42ba0cb158e02e7111d56f41682fde` |
| D435 엔진 | `~/yolo_D/deploy/yolo11n_task4_E3_480_fp16.engine`, 입력 1×3×480×640, FP16, sha256 `d7cea24d68d94bd94ba27995f14deb3105d90dbac9b97f0a3fb3df1f2b58345f` |
| 팔 엔진 | `~/yolo_D/deploy/yolo11n_task4_E3_384_fp16.engine`, 입력 1×3×384×640, FP16, sha256 `5d09c3620eb9bc66370edba2c7bf6197088375f91a95b1caf27f60b4ad2bb0d3` |
| ONNX | 480 `d82ab8670d014722f2d70335b25f562c075c6c79747a228478347d2f571693b2`, 384 `d0ced668034580927711fe2fdd712cefdc14da0a6a803f42d581363cb8002071` |
| 환경 | L4T R39.2.1, TensorRT 10.16.2, nvpmodel 25W |
| 설정 | `perception/config/perception.yaml`: `detector.engine_path` · `arm.engine_path` |

점수 문턱은 쓰는 곳마다 다르다.

| 문턱 | 값 | 적용 대상 |
|---|---|---|
| `detector.engine_score_threshold` | 0.25 | 엔진 출력 하한. 앞뒤 D435와 팔 모두 |
| `detector.nms_threshold` | 0.45 | 겹침 제거(클래스마다) |
| `target.min_confidence` | 0.5 | D435 목표 고르기 |
| 등록부 D435 관측 | 0.6 | 노드 안 고정값 |
| `tagged_min_confidence` | 0.4 | 태그 퍽 색 · 모양 점수. YOLO 점수가 아니라 색띠 점유율 |
| YOLO 판정 | 0.6 | 재현율 · 정밀도 95 % 판정에 쓴 값 |

## 2. 기동과 정상 확인

**1. 엔진 파일 확인** (필수, 엔진이 없으면 경고만 남기고 색 검출기로 돈다) — Jetson
```bash
sha256sum ~/yolo_D/deploy/yolo11n_task4_E3_480_fp16.engine ~/yolo_D/deploy/yolo11n_task4_E3_384_fp16.engine
```
1절의 sha256과 같아야 한다. `fix/perception-engine-hash` 가 합쳐진 뒤에는 시작 로그에 `해시 일치` 가 보여야 한다(`해시 파일 없음` 이면 검사하지 않은 것이다).

현황(2026-11-06): Jetson 에 해시 파일 2개를 만들었고 `sha256sum -c` 는 OK 다. Jetson 의 perception 빌드는 해시 코드가 없는 시험 사본뿐이라 `해시 일치` 로그는 아직 보지 못했다. dev 를 배포한 뒤 확인한다.

**2. 전력 모드 확인** (필수, 측정값은 모두 25W 기준) — Jetson
```bash
nvpmodel -q
```

**3. 기동** (필수) — Jetson. 전체 기동은 `bringup.launch.py`, 인지만 띄울 때는 아래 명령을 쓴다.
```bash
ros2 launch perception perception.launch.py
```

**4. 시작 로그 확인** (필수) — Jetson, 기동 터미널

`검출기:` 로 시작하는 줄에 `yolo11n_task4_E3_480_fp16.engine` 과 팔 쪽 E3 384 엔진이 보여야 한다. 색 검출기로 대신한다는 WARN 이 있으면 정상이 아니다(3절).

**5. 결과 토픽 확인** (필수) — Jetson
```bash
ros2 topic hz /perception/pucks
```
결과가 계속 나와야 한다. 통합 상태의 정상 주기는 아직 재지 않았다(5절).

## 3. 오류별 조치

| 보이는 것 | 뜻 | 조치 |
|---|---|---|
| WARN `detector.engine_path 를 쓰지 못해 색 검출기로 대신함` | D435가 YOLO 없이 돈다. 노드는 멈추지 않는다 | 뒤에 붙은 이유를 보고 아래 줄대로 고친 뒤 다시 기동 |
| WARN `arm.engine_path 를 쓰지 못해 팔도 주 검출기를 씀` | 팔이 480 엔진으로 돈다. 입력이 달라 팔 판정 결과가 맞지 않는다 | 경로와 해시 확인 |
| `엔진 해시 불일치` · `엔진 해시 파일 형식이 틀림` | 엔진이 변조됐거나 다른 엔진이다. 모르고 쓰면 경고 없이 퍽을 못 찾는다(장애 주입 F12) | 원본에서 다시 복사. 해시 파일을 현재 파일로 다시 만들지 말 것 |
| `엔진 파일을 열 수 없음` | 경로가 틀렸거나 파일이 없다 | 1절 경로에 복사 |
| `엔진 역직렬화 실패` | 엔진을 만든 TensorRT · GPU와 다르다 | 같은 ONNX로 그 Jetson에서 다시 빌드(4절 뒤) |
| `입력 1개 · 출력 1개 엔진만 씀` | NMS가 들어간 엔진이다 | NMS 없는 ONNX로 다시 빌드 |
| `CUDA 메모리 할당 실패` | GPU 메모리가 모자란다 | 다른 GPU 작업을 끄고 다시 기동 |
| FATAL `arm.engine_path 클래스 수 … 가 주 검출기 … 와 다름` | 두 엔진의 클래스 수가 다르다. 노드가 시작하지 않는다 | 같은 가중치의 480 · 384 엔진을 쓴다 |
| `CAMERA_STALL` | 그 모드에 쓸 수 있는 새 영상이 없다. 결과 발행이 멈춘다 | 카메라 드라이버와 토픽 확인. 영상이 돌아오면 스스로 복구(모의 0.2 s 안) |
| `ARM_TF_PENDING` | 팔 영상 촬영 시각의 TF 가 아직 없다. 팔 결과가 안 나온다 | `/joint_states` 와 관절 수 확인 |
| 결과가 끊기고 프로세스가 없다 | 인지 컨테이너가 죽었다. 자동 재시작이 없고 launch 는 남아 있다 | launch 를 끄고 다시 기동. 모드를 다시 보내야 결과가 나온다 |
| trtexec 경고 `different models of devices` | 모든 엔진에서 나왔고, 같은 엔진의 판정이 한 자리까지 같았다 | 조치 없음 |
| 신호등을 퍽으로 잡는다 | 판정에서 헛상자로 세는 항목이다 | 장면을 저장해 YOLO 쪽에 알린다 |

주 엔진이 실패하면 팔 엔진이 멀쩡해도 무시되고 팔도 색 · 라벨 검출로 돈다. 위 WARN 이 하나라도 있으면 고친 뒤 다시 기동한다.

엔진 다시 빌드 (필수, JetPack · TensorRT를 바꿨을 때만) — Jetson. ONNX를 `~/e3_check/` 에 둔 상태에서 실행한다. `480` 을 `384` 로 바꿔 한 번 더 돌린다.
```bash
/usr/src/tensorrt/bin/trtexec --onnx=$HOME/e3_check/yolo11n_task4_E3_480.onnx --fp16 --saveEngine=$HOME/yolo_D/deploy/yolo11n_task4_E3_480_fp16.engine --skipInference
```
다시 빌드하면 해시가 바뀔 수 있다. 그 경우 `check_e3.py` 와 `analyze_e3.py` 로 정합을 다시 확인한다(K1 ≥ 98 %, K2 ±0.01). 통과한 뒤에 `<엔진>.sha256` 을 새로 만들고 1절 값을 고친다.

## 4. 이전 엔진으로 되돌리기

되돌리면 팔 성능이 내려간다. 경기장 세트 @0.6 기준이다. E3 는 Jetson 엔진, E2 · D 는 PC pt 값이다(E3 뒤 카메라는 PC pt 0.981).

| 엔진 | 앞 | 뒤 | 팔 | sha256 앞 8자 (480 · 384) |
|---|---|---|---|---|
| E3 | 0.974 · 0.990 | 0.979 · 0.998 | 0.956 · 0.981 | d7cea24d · 5d09c362 |
| E2 | 0.954 · 0.987 | 0.9485 · 0.998 | **0.871 · 0.948 (불통과)** | 315b0608 · 15d8bb9c |
| D | 0.878 · 0.975 (불통과) | 0.929 · 0.988 (불통과) | 0.749 · 0.871 (불통과) | 58df0823 · e22717e8 |

원본 설정은 바꾸지 않고 사본을 쓴다.

**1. 설정 사본 만들기** (필수, 되돌릴 때만) — Jetson
```bash
mkdir -p ~/perception_cfg_e2 && cp $(ros2 pkg prefix perception)/share/perception/config/*.yaml ~/perception_cfg_e2/
```

**2. 사본의 엔진 경로 바꾸기** (필수, 되돌릴 때만) — Jetson
```bash
sed -i 's/yolo11n_task4_E3_/yolo11n_task4_E2_/' ~/perception_cfg_e2/perception.yaml
```

**3. 사본으로 기동** (필수, 되돌릴 때만) — Jetson
```bash
ros2 launch perception perception.launch.py config_dir:=$HOME/perception_cfg_e2
```
시작 로그에 E2 경로가 보이는지 확인한다. `bringup.launch.py` 로 기동할 때 사본을 넘기는 방법은 아직 확인하지 않았다.

## 5. 확인한 것과 안 한 것

| 항목 | 상태 |
|---|---|
| 엔진 정합(같은 장 99.82 %, 판정 차 ≤ 0.002) | 실물 Jetson 확인, 11-04 · 11-06 두 번 |
| GPU 시간 480 · 384 (3.25 · 2.98 ms) | 실물 Jetson 확인, 세 번. 카메라 없이 잰 값 |
| 인지 통합 모의(A1 · F1) | Isaac 렌더 영상으로 통과. 실물 아님 |
| Jetson `~/yolo_D/deploy/` 의 E3 해시 | 복사 때 오류 없음만 확인. 해시는 2절 1번으로 확인할 것 |
| 처음 보는 영상에서의 정확도 | **미검증.** 최종 영상 촬영 전. 경기장 밖 같은 조명 · 바닥에서 찍을 예정이라 경기장 배경의 새 영상 성능은 계속 미검증 |
| E3 부하(지연 · GPU · 온도) | **미검증.** 480 · 384 FP16 부하는 D 엔진으로 GPU 18.9 %, 14.83 Hz. 30분 소크는 FP32 640 엔진으로 통과. 다시 재지 않기로 함(사용자 결정) |
| 팔 1280×720 | **미검증.** bringup 은 아직 640×480(Issue #119 열림). 판정은 1280×720 영상으로만 했다 |
| 팔 실행 문턱 | 판정은 0.6, 실행은 0.25. 아래 표 참고 |
| 장애 주입 12항목(카메라 · 깊이 · TF 중단, 지연, 깨진 영상, 일시 정지, 프로세스 종료, 엔진 손상) | PC 모의 통과(`261106_1130_fault_inject`). 실물 · Jetson 미검증, 제어기 반응은 코드만 읽음 |
| 엔진 해시 검사 | 코드와 PC 모의 통과, PR 전. Jetson 해시 파일 없음 |
| 이 문서의 명령 순서, 되돌리기 | **미실행** |

E3 팔 판정을 점수 문턱별로 본 값이다(Jetson 엔진, 경기장 세트, 재현율 · 정밀도).

| 문턱 | 앞 | 뒤 | 팔 |
|---|---|---|---|
| 0.25 (실행값) | 0.989 · 0.977 | 0.991 · 0.983 | 0.982 · **0.930** |
| 0.5 | 0.980 · 0.987 | 0.985 · 0.992 | 0.974 · 0.957 |
| 0.6 (판정값) | 0.974 · 0.990 | 0.979 · 0.998 | 0.956 · 0.981 |

D435 등록부는 0.6으로 다시 거르지만, 팔 정렬(ARM_ALIGN)은 0.25 상자를 D435 목표 투영점 근처로 잠가서 쓴다. 그래서 위 0.930이 그대로 잘못된 잠금 비율은 아니다. 실제 영향은 실물 팔 시험에서 확인해야 한다.
