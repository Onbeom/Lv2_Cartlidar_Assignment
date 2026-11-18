# 로봇 배포 방법 (2026-11-06, 기준 origin/dev b71aa21)

bb6704c 뒤 변경(#134 · #136 · #137 · #138)에 `opencr_firmware` 는 없어 `fw_check/out` 펌웨어 빌드는 그대로 쓸 수 있다. 저장소를 clone 하면 그때의 dev 를 받으니 시작 전에 SHA 를 적어 둔다.

**Jetson에서 아직 이 순서로 돌려 보지 않았다.** PC에서 확인한 것은 맨 아래 표를 본다.

## 무엇이 어디로 가나

| 대상 | 저장소에 있나 | 실행 파일 | 어디서 돌아가나 | 배포 방법 |
|---|---|---|---|---|
| 통합 · 주행 · 인지 · 팔 제어 코드 | 있음 | colcon 빌드 결과 | Jetson | git clone → apt 설치 → colcon build |
| 모터 구동(펌웨어) | 있음(colcon 제외) | `.bin` 2개 | OpenCR 2대 | PC에서 빌드 · USB 업로드 |
| 모터 구동(호스트 쪽 `opencr_hardware`) | 있음 | colcon 빌드 결과 | Jetson | 위 colcon 빌드에 포함 |
| YOLO E3 | **없음**(pt · onnx · engine 모두) | `.engine` 2개 | Jetson GPU(TensorRT) | Jetson `~/yolo_D/deploy/`에 이미 있음. 저장소에는 경로만 |
| 집기 강화학습 TopPick | 있음(`toppick_tilt.onnx`) | `.onnx` | Jetson CPU(ONNX Runtime) | ONNX Runtime 설치 → 빌드 → 설정 켜기. 기본 꺼짐 |
| 주행 강화학습 | **배포할 파일 없음** | - | - | 실물 주행은 Nav2. `slalom_v1.onnx` 없음, #113 열림 |

YOLO 파일 흐름: `.pt`(학습) → `.onnx`(중간 형식) → `.engine`(TensorRT 실행 파일). 인지 C++ 노드는 `.engine`만 읽는다. `.engine`은 만든 TensorRT · GPU에 묶여 있어 JetPack을 바꾸면 ONNX에서 다시 만든다(`261106_1122_yolo_runbook/RUNBOOK.md` 3절).
PC 사본: `261104_1832_E3_deploy/yolo11n_task4_E3_{480,384}.onnx`, `261104_1832_E3_deploy/results/*.engine`(sha256이 RUNBOOK 값과 같음, 11-06 PC 확인), `261104_1618_E3/runs/E3/weights/last.pt`.

## A. 지금 할 수 있는 것 (로봇 없이)
순서: 도구 복사 → 점검 → clone → apt → ONNX Runtime → 빌드 → 인지 해시 확인 → teleop 경로 확인. 명령은 채팅과 같다.

```bash
rsync -av ~/git/lv2_next_module/261106_1655_real_drive/jetson/ cart11@10.2.17.245:~/real_drive/
```
```bash
bash ~/real_drive/check_deploy.sh 2>&1 | tee ~/real_drive/check_result.txt
```
```bash
git clone -b dev --single-branch https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment.git ~/Lv2_Cartlidar_Assignment
```
```bash
grep -v '^#' ~/real_drive/requirements.txt | grep -v '^$' | xargs sudo apt-get install -y
```
PC 에 받아 둔 `onnxruntime/onnxruntime-linux-aarch64-1.26.0.tgz`(sha256 34ff1c2d…)를 Jetson 으로 보낸다. 팔 · 주행 시험에는 필요 없다.
```bash
rsync -av ~/git/lv2_next_module/261106_1655_real_drive/onnxruntime/onnxruntime-linux-aarch64-1.26.0.tgz cart11@cart11.local:~/real_drive/
```
```bash
mkdir -p ~/onnxruntime && tar xzf ~/real_drive/onnxruntime-linux-aarch64-1.26.0.tgz -C ~/onnxruntime --strip-components=1
```
```bash
cd ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws && source /opt/ros/lyrical/setup.bash && MAKEFLAGS=-j2 colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DONNXRUNTIME_ROOT=$HOME/onnxruntime
```
```bash
bash ~/real_drive/check_perception.sh
```

## B. 조립 뒤 (보드 · 모터 필요)
모터 ID(바퀴 1~4, 팔 11~16)는 `tools/dxl_setup`으로 한 대씩 맞춘다. 6륜 때 ID가 남아 있으면 3→2, 4→3, 6→4 순서(`opencr_firmware/README.md`). 그리퍼 PWM Limit은 400 이하(`w 15 400`), 아니면 부팅 점검에서 FAULT.

펌웨어는 PC에서 올린다. 이 PC의 OpenCR 업로드 도구(`opencr_ld`)는 x86-64 판뿐이라 Jetson에서는 쓸 수 없다. 보드 하나씩 PC에 USB로 연결한다. 잘못된 역할을 올리면 모터가 엉뚱하게 움직일 수 있다.

```bash
ARDUINO_CLI=$HOME/Desktop/arduino-ide_2.3.10_Linux_64bit/resources/app/lib/backend/resources/arduino-cli OPENCR_BUILD_DIR=$HOME/git/lv2_next_module/261106_1655_real_drive/fw_check/out bash $HOME/git/lv2_next_module/261106_1655_real_drive/fw_check/workspace/ros2_ws/src/opencr_firmware/tools/build_firmware.sh wheel --upload /dev/ttyACM0
```
팔 보드는 위 명령의 `wheel`을 `arm`으로 바꾼다. 멈추면 Recovery Mode: SW2를 누른 채 Reset을 눌렀다 떼고 SW2를 뗀다.

확인(보드를 Jetson에 다시 연결한 뒤, role=1 주행 · role=2 팔):
```bash
python3 ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/ttyACM0 --seconds 2
```
OpenCR 포트 규칙과 권한(sudo):
```bash
sudo cp ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/udev/99-opencr.rules /etc/udev/rules.d/
```
```bash
sudo usermod -aG dialout cart11
```
센서 규칙 `perception/udev/99-sensors.rules`는 TFmini · 라이다 시리얼 칸이 비어 있어(★) 실물 값을 넣은 사본을 쓴다.

첫 기동(S1, 바퀴를 띄운 상태, 주행만):
```bash
source ~/real_drive/env.sh && ros2 launch bringup bringup.launch.py use_perception:=false use_sensors:=false use_tfmini:=false
```
키보드 조종(실제 조이스틱은 뽑고, 다른 터미널에서):
```bash
source ~/real_drive/env.sh && python3 ~/real_drive/keyboard_teleop.py
```

## C. 코드가 바뀌면
```bash
git -C ~/Lv2_Cartlidar_Assignment pull
```
그 뒤 A의 빌드 명령을 다시 친다. 펌웨어는 `opencr_firmware`가 바뀌었을 때만 다시 올린다(프로토콜이 다르면 호스트가 연결을 거부).

## D. 강화학습 집기를 켤 때
`skill_enabled:=true`만으로는 부족하다. `manipulator.yaml`의 `skill.policies`가 아직 `[pick_cup_v1]`이고 그 파일은 저장소에 없다. 설정 사본에서 `toppick_tilt`를 넣어야 한다. 담당 WindForce08, 관련 #78 · #108. 실물 시험 전.

## 확인한 것과 안 한 것

| 항목 | 상태 |
|---|---|
| 펌웨어 빌드 wheel · arm · setup | PC 빌드 통과(bb6704c, f036e42 와 펌웨어 소스 같음). 업로드 · 실물 동작은 안 함. README도 주행 보드는 실물 미시험이라고 적음 |
| 키보드 teleop | PC 모의 15/15(`teleop_check/result.txt`), 팀 EstopLatch 코드에 재생해 잠김 · 풀림 확인. `joy_node` · `teleop_twist_joy` · `twist_mux`를 거친 길은 안 함(PC에 없음) |
| apt 패키지 이름 | PC(amd64) 후보로 확인. Jetson(arm64)은 `check_deploy.sh`의 모의 설치로 확인 |
| ONNX Runtime 1.26.0 aarch64 | 릴리스 파일 있음만 확인. Jetson 빌드 · 정책 실행은 안 함 |
| Jetson 전체 colcon 빌드, 메모리 | 안 함. `-j2`는 8 GB 보드 메모리를 고려한 제안 |
| `cartographer_ros` | Lyrical apt 후보 없음. Jetson에는 안 깔고, 실물 지도는 PC 오프라인으로 만든다(S4). 기본 위치 추정은 AMCL이라 영향 없음 |

## E. 바퀴 보드 = PC, 팔 보드 = Jetson 구성 (빌드 없음)
`jetson/` 전체를 Jetson 으로 복사한 뒤 PC 에서 `python3 ~/git/lv2_next_module/261106_1655_real_drive/jetson/robot_teleop.py --arm_ssh cart11@cart11.local`. 펌웨어 업로드는 x86-64 도구라 두 보드 모두 PC 에 꽂아서 한다. 키는 실행 화면에 나온다.
