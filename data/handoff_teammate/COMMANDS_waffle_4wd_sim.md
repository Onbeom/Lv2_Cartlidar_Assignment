# Waffle 4WD 시뮬레이션 인수인계 — 환경 확인·설치·파일 확인 명령 모음

작성: 2026-10-21, 갱신: 2026-10-22(로봇 모델 261022 리비전), 2026-10-22 18:00(261022_1800 리비전: 라이다 scan 프레임 roll π, RealSense 적외선·컬러 광학 프레임 6개 추가), 2026-10-23(261022_2119 리비전: 관절 속도 한계를 ROBOTIS 공식 무부하 속도로), 2026-10-27(261027_1530 리비전: 층간 긴 브라켓 위상최적화, 저장소 구멍판, Arducam 광축 15°, 증속 벨트 1:1.5), 2026-10-27 저녁(261027_1711 리비전: 새 rail, 앞 무게추 없음, 출력 부품 채움률 15 %). 이 문서와 함께 받는 파일은 세 개다.

| 파일 | 내용 | SHA-256 |
|---|---|---|
| `urdf.zip` | 원본 URDF 패키지 `waffle_4wd_description_261021`(23링크·22관절, 메시 504개)와 CAD 내보내기 기록(`validation.md` 등). 풀면 `urdf/` 폴더가 쓰기 금지 권한으로 만들어진다 | `8c858fad4c1f82161e638b12618f290d236c32fb00a72a2f4dd04aae25b2bba7` |
| `urdf_261027_1711.zip` | **실제로 쓰는 URDF 패키지 261027_1711 리비전**(`urdf_261027_1711/waffle_4wd_description_261027_1711/`, 패키지 이름 `waffle_4wd_description_261027_1711`, 29링크·28관절). 원본과 다른 것: ① `arm_link_5.dae`(표시 메시)에서 CAD 내보내기 때 잘못 들어간 집게 체결 나사 4세트를 지움(`REVISION_261022.md`) ② 라이다 scan 프레임에 roll π(라이다가 3층 판 아래에 거꾸로 매달림) ③ RealSense 앞·뒤에 `*_infra1/infra2/color_optical_frame` 추가(D435 공장 보정값) ④ 관절 속도 한계를 ROBOTIS 공식 무부하 속도(11.1 V)로: 팔 0.5 → 4.502949 rad/s(XM430-W350 43 rev/min), 집게 0.01 m/s 그대로(`REVISION_261022_2119.md`) ⑤ 층간 긴 브라켓 4개 위상최적화(부피 −20 %), 저장소 housing·magazine 구멍판, Arducam 광축을 장착면 기준 15°로(`camera_link_fixed` pitch 1.308997 rad), 증속 벨트(GT2 36T → 24T, 1 : 1.5) 4모듈과 앞 무게추: 바퀴 관절 y ±0.1396 → ±0.1546 m, 바퀴 속도 한계 10.995574 rad/s(70 rev/min × 1.5)·effort 1.8 N·m, base_link 3.7218 → 4.6156 kg(총 5.7850 kg), 충돌 상자 9개 추가(베어링 블록 4·벨트 덮개 4·무게추 1)(`REVISION_261027_1711.md`) ⑥ 패키지 이름. 팔·집게의 질량·관절 각도 한계·effort는 그대로 ⑦ (261027_1711) 저장소 rail을 새 STEP(깊이 87 mm)으로 바꿈 · 충돌은 CoACD 볼록 분해, 앞 무게추(컵 · 강철) 없음, 출력 부품 질량에 채움률 15 %(튜터) + 껍질 0.8 mm(가정) 반영 → 총질량 4.2207 kg | `f49b7c1434499310f20fdbc11a63e0406372d4000ddb53f0984f39ba2ec35be2` |
| `waffle_4wd_usd_261027_1711.zip` | 261027_1711 URDF를 Isaac Sim 6.1 importer로 변환한 USD 패키지(폴더째: `waffle_4wd.usda` + `payloads/`) | `d0da1f628c54257ac654a69a70161d25521552e9fdccc9258a0dfd243e80b300` |

AI와 함께 진행할 때는 이 문서 전체를 AI에게 먼저 읽히고, 아래 "6. 현재 상태와 남은 일"부터 이어 가게 한다.

## 사용 규칙

- 각 명령 앞에 **(필수)/(선택 · 권장)/(선택 · 비권장)** 과 실행 위치를 적었다. 실행 위치는 모두 **PC(Ubuntu 24.04, NVIDIA GPU 직결)** 다.
- 코드 블록은 **안쪽 명령만** 복사해 터미널에 붙여 넣는다. 문서 본문(```로 시작하는 줄 포함)을 통째로 붙여 넣으면 bash가 백틱을 명령 치환으로 해석해 셸이 이상해진다.
- `sudo` 비밀번호는 직접 입력한다.
- `export`로 지정한 환경 변수는 그 터미널에서만 유지된다. 새 터미널마다 다시 지정하거나 `~/.bashrc`에 넣는다.

## 1. 기준 환경 (작성자 PC에서 실제 확인한 값)

| 항목 | 값 |
|---|---|
| OS·커널 | Ubuntu 24.04.5 LTS, 6.8.0-147-generic |
| GPU·드라이버 | NVIDIA GeForce RTX 5060 Laptop 8151 MiB, 드라이버 595.99.02, CUDA 13.2 표시 |
| Python | 3.12.3 |
| Isaac Sim | pip `isaacsim[all,extscache]==6.1.0.0`, torch 2.11.0+cu130, venv `~/isaacsim-venv` |
| Isaac Sim URDF importer | `isaacsim.asset.importer.urdf` 3.11.10 (URDF USD Converter v0.3.2) |
| Isaac Lab | `v3.0.0-EA` 커밋 `ae37b028ea415c91ea2bc32609efcd759ed2b974`, `uv sync --frozen --extra isaacsim`(uv 0.13.1), Lab venv torch 2.11.0+cu128, isaaclab 17.0.2, rsl-rl-lib 5.4.1 |
| ROS 2 | Lyrical(apt). ros-base 0.13.0, rviz2 15.2.5, robot-state-publisher 3.5.5, joint-state-publisher-gui 2.4.3, xacro 2.1.1, rmw-cyclonedds-cpp 4.1.5, moveit 2.15.0, moveit-setup-assistant 2.15.0 |
| ROS 환경 변수 | `ROS_DOMAIN_ID=26`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| (선택) Isaac ROS | 5.0, `ros-lyrical-isaac-ros-cuvslam` 5.0.0 — 이번 인수인계 범위(URDF·USD·MoveIt)에는 필요 없다 |

공식 최소 사양은 RAM 32 GB·VRAM 16 GB다. 작성자 PC(VRAM 8 GB)는 호환성 검사에서 VRAM 항목만 FAILED였고 실행은 됐다.

## 2. 환경 확인 명령

(필수) OS 버전 확인 · PC
```bash
cat /etc/os-release
```
(필수) GPU·드라이버·VRAM 확인 · PC — GPU 이름과 드라이버가 나와야 한다. 드라이버 자동 갱신 직후에는 재부팅이 필요할 수 있다.
```bash
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
```
(필수) Python 3.12 확인 · PC
```bash
python3.12 --version
```
(필수) 디스크 여유 확인 · PC — Isaac Sim venv 약 26 GB, Isaac Lab venv, 메시 등을 합쳐 50 GB 이상 여유를 권장한다.
```bash
df -h "$HOME"
```
(필수) ROS 2 Lyrical 설치 확인 · PC — `/opt/ros/lyrical`이 출력되면 설치돼 있다.
```bash
ls -d /opt/ros/lyrical
```
(필수) 필요한 ROS 패키지 확인 · PC — 각 줄에 버전이 나오면 설치돼 있다. 빠진 것은 3장에서 설치한다.
```bash
dpkg-query -W -f='${Package} ${Version}\n' ros-lyrical-ros-base ros-lyrical-rviz2 ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher ros-lyrical-joint-state-publisher-gui ros-lyrical-xacro ros-lyrical-urdfdom ros-lyrical-rmw-cyclonedds-cpp ros-lyrical-moveit ros-lyrical-moveit-setup-assistant python3-colcon-common-extensions
```
(필수) Isaac Sim pip 설치 확인 · PC — `isaacsim 6.1.0.0`이 나와야 한다.
```bash
~/isaacsim-venv/bin/python -c "from importlib.metadata import version as v; print('isaacsim', v('isaacsim'), 'torch', v('torch'))"
```
(선택 · 권장) Isaac Lab 확인 · PC — 커밋이 `ae37b028ea415c91ea2bc32609efcd759ed2b974`여야 한다. 강화학습 단계에서 필요하다.
```bash
git -C ~/pa-course/IsaacLab-6.1 rev-parse HEAD
```

## 3. 설치 (없을 때만)

### 3-1. 기본 도구

(필수) 빌드 도구·Python venv · PC
```bash
sudo apt install -y git curl build-essential cmake python3.12-venv python3-pip python3-colcon-common-extensions
```

### 3-2. ROS 2 Lyrical

이미 `/opt/ros/lyrical`이 있으면 건너뛴다. 없으면 ROS 공식 설치 문서(Lyrical, Ubuntu 24.04)의 apt 저장소 설정을 따른다. 작성자 PC는 `http://packages.ros.org/ros2/ubuntu noble main` 저장소를 쓴다.

(필수) 이번 작업에 필요한 ROS 패키지 · PC
```bash
sudo apt install -y ros-lyrical-ros-base ros-lyrical-rviz2 ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher ros-lyrical-joint-state-publisher-gui ros-lyrical-xacro ros-lyrical-urdfdom ros-lyrical-rmw-cyclonedds-cpp
```
(필수) MoveIt 2(충돌 확인용) · PC
```bash
sudo apt install -y ros-lyrical-moveit ros-lyrical-moveit-setup-assistant
```
(선택 · 권장) 매 터미널 ROS 환경 자동 설정 · PC — 같은 네트워크의 다른 ROS와 섞이지 않게 도메인을 정한다. 팀이 다른 번호를 쓰면 그 값으로 바꾼다.
```bash
printf '%s\n' 'source /opt/ros/lyrical/setup.bash' 'export ROS_DOMAIN_ID=26' 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

### 3-3. Isaac Sim 6.1 (pip, 전용 venv)

(필수) venv 생성 · PC — 이미 `~/isaacsim-venv`가 있으면 건너뛴다.
```bash
python3.12 -m venv ~/isaacsim-venv
```
(필수) pip 갱신 · PC
```bash
~/isaacsim-venv/bin/python -m pip install --upgrade pip
```
(필수) torch(CUDA 13 빌드) · PC
```bash
~/isaacsim-venv/bin/python -m pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu130
```
(필수) Isaac Sim 6.1.0.0 · PC — 크기가 크다(약 26 GB).
```bash
~/isaacsim-venv/bin/python -m pip install 'isaacsim[all,extscache]==6.1.0.0' --extra-index-url https://pypi.nvidia.com
```
(필수) 의존성 충돌 확인 · PC — `No broken requirements found.`가 나와야 한다.
```bash
~/isaacsim-venv/bin/python -m pip check
```
(필수) 첫 실행과 NVIDIA 약관 동의 · PC — 창이 뜨면 약관을 직접 읽고 동의한 뒤 창을 닫는다.
```bash
~/isaacsim-venv/bin/isaacsim
```

### 3-4. Isaac Lab (강화학습 단계에서 필요)

(선택 · 권장) uv 설치 · PC — 이미 `uv --version`이 되면 건너뛴다.
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
(선택 · 권장) 고정 버전 받기 · PC
```bash
git clone --depth 1 --branch v3.0.0-EA https://github.com/isaac-sim/IsaacLab.git ~/pa-course/IsaacLab-6.1
```
(선택 · 권장) 커밋 확인 · PC — `ae37b028ea415c91ea2bc32609efcd759ed2b974`가 아니면 진행하지 않는다.
```bash
git -C ~/pa-course/IsaacLab-6.1 rev-parse HEAD
```
(선택 · 권장) Lab 전용 환경 설치 · PC — `uv.lock`을 그대로 쓰므로 작성자와 같은 의존성 버전이 설치된다.
```bash
cd ~/pa-course/IsaacLab-6.1 && uv sync --frozen --extra isaacsim
```

## 4. 받은 파일 배치·확인

(필수) 작업 폴더 만들기 · PC
```bash
mkdir -p ~/waffle_sim/sim_ws/src ~/waffle_sim/sim/isaac/usd ~/waffle_sim/sim/tools
```
(필수) 받은 파일 해시 확인 · PC — 1행 표의 SHA-256과 같아야 한다. 다르면 파일이 바뀐 것이니 진행하지 않는다.
```bash
sha256sum ~/Downloads/urdf.zip ~/Downloads/urdf_261027_1711.zip ~/Downloads/waffle_4wd_usd_261027_1711.zip
```
(필수) 원본 URDF·CAD 기록 압축 해제 · PC — 미검증 목록(`validation.md`) 등 기록을 보기 위해 푼다.
```bash
unzip -q ~/Downloads/urdf.zip -d ~/waffle_sim
```
(필수) 261027_1711 URDF 압축 해제 · PC — `~/waffle_sim/urdf_261027_1711/`에 풀린다(원본 `urdf/` 폴더는 쓰기 금지라 따로 둔다).
```bash
unzip -q ~/Downloads/urdf_261027_1711.zip -d ~/waffle_sim
```
(필수) USD 압축 해제 · PC — `waffle_4wd/` 폴더째 써야 한다(`waffle_4wd.usda`가 `payloads/`를 상대 경로로 참조한다).
```bash
unzip -q ~/Downloads/waffle_4wd_usd_261027_1711.zip -d ~/waffle_sim/sim/isaac/usd
```
(필수) 파일 확인 · PC — `waffle_4wd.urdf`, `waffle_4wd.urdf.xacro`, `waffle_4wd.usda`, `payloads`가 보여야 한다.
```bash
ls ~/waffle_sim/urdf_261027_1711/waffle_4wd_description_261027_1711/urdf ~/waffle_sim/sim/isaac/usd/waffle_4wd
```
(필수) URDF 패키지를 작업 공간에 연결 · PC
```bash
ln -s ~/waffle_sim/urdf_261027_1711/waffle_4wd_description_261027_1711 ~/waffle_sim/sim_ws/src/waffle_4wd_description_261027_1711
```
(필수) MoveIt 설정 패키지·도구 스크립트 만들기 · PC — 이 문서 끝 "부록 A"의 각 파일을 적힌 경로에 같은 내용으로 저장한다(AI에게 "부록 A대로 파일을 만들어 줘"라고 요청하면 된다).

(필수) 빌드 · PC
```bash
cd ~/waffle_sim/sim_ws && source /opt/ros/lyrical/setup.bash && colcon build
```

## 5. 확인 명령 (작성자 PC 결과와 비교)

모든 ROS 명령 전에 같은 터미널에서 먼저 실행한다(필수 · PC):
```bash
source ~/waffle_sim/sim_ws/install/setup.bash
```

### 5-1. URDF 정적 확인 (작성자 결과: 모두 성공)

(필수) 파싱·트리 · PC — `Successfully Parsed XML`, root `base_footprint`, 링크 29개 트리가 나와야 한다.
```bash
check_urdf ~/waffle_sim/urdf_261027_1711/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf
```
(필수) xacro 펼치기 · PC — 오류 없이 끝나야 한다. 작성자 PC에서는 펼친 결과와 `.urdf`의 링크·관절 이름·수치 차이가 0이었다.
```bash
xacro ~/waffle_sim/urdf_261027_1711/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf.xacro > /tmp/waffle_4wd_expanded.urdf
```

### 5-2. RViz 표시·TF (작성자 결과: 모두 성공)

(필수) 표시 실행 · PC — RViz와 관절 슬라이더 창이 뜬다. 슬라이더는 10개(바퀴 4, 팔 5, 왼쪽 집게 1. 오른쪽 집게는 왼쪽의 −1 mimic이라 슬라이더 없음).
```bash
ros2 launch waffle_4wd_description_261027_1711 display.launch.py
```
(필수) TF 확인 · PC(다른 터미널) — 관절 0 자세에서 작성자 값: `camera_optical_frame` (0.336, 0, 0.374) m, `top_camera_front_optical_frame` (−0.061, 0, 0.490), `top_camera_rear_optical_frame` (−0.220, 0, 0.490), `lidar_front_scan` (0.185, 0.050, 0.068), `lidar_rear_scan` (−0.185, −0.050, 0.068). 두 scan 프레임은 z축이 아래(라이다가 거꾸로 매달림)이고, `lidar_front_scan`의 x축은 앞, `lidar_rear_scan`의 x축은 뒤다. RealSense 적외선 두 프레임 간격은 0.0500 m다.
```bash
ros2 run tf2_ros tf2_echo base_footprint camera_optical_frame
```

### 5-3. MoveIt 충돌 확인 (작성자 결과: 모두 성공)

(필수) 실행 · PC — RViz MotionPlanning > Joints 탭에서 팔을 움직이면 충돌한 링크가 빨간색이 된다. 로그의 ERROR 3건(3D 센서 없음, 컨트롤러 이름 없음, `/recognize_objects` 없음)은 일부러 넣지 않은 기능이라 정상이다.
```bash
ros2 launch waffle_4wd_moveit_config collision_check.launch.py
```
(필수) 자세별 충돌 검사 · PC(다른 터미널) — 인자는 [yaw, shoulder, elbow, wrist pitch, wrist roll] rad. 작성자 결과(261027_1711): 관절 0 자세, `0 1.4992378274631293 -1.3997540600994522 1.6999506914424771 0`, 주행 접은 자세 `0 1.4 -1.0 -1.3 0`은 충돌 0쌍, `0 1.5 1.5 1.7 0`은 base_link–arm_link_3(0.0084 m), base_link–arm_link_4(0.0017 m), gripper_right_link–lidar_rear(0.0148 m).
```bash
python3 ~/waffle_sim/sim/tools/check_arm_collision.py 0 1.5 1.5 1.7 0
```

### 5-4. USD (작성자 결과: 261027_1711 리비전으로 생성 성공, 판정 ②~⑩ 성공 — ⑤·⑩은 조건부, 충돌 형상 C1~C3 성공)

(선택 · 권장) USD를 Isaac Sim 창에서 열기 · PC — Isaac Sim이 뜨면 File > Open으로 `~/waffle_sim/sim/isaac/usd/waffle_4wd/waffle_4wd.usda`를 연다. 이 USD는 Physics variant 기본 선택이 비어 있어 그냥 열면 질량·관절 없이 모양만 열린다. Stage 창에서 `waffle_4wd` prim을 고른 뒤 Property 창의 Variants에서 Physics를 `physx`로 고른다(Isaac Lab에서는 스폰 설정에 `variants={"Physics": "physx"}`).
```bash
~/isaacsim-venv/bin/isaacsim
```
(선택) USD를 같은 설정으로 다시 만들기 · PC — 받은 USD를 쓰면 필요 없다. 부록 A의 `import_waffle_4wd.py`가 필요하다. 결과는 `~/waffle_sim/sim/isaac/usd_rebuild/waffle_4wd/waffle_4wd.usda`. 경고 "Stiffness and damping not available ..." 11줄은 강성·감쇠를 아직 정하지 않아서 나오는 정상 경고다.
```bash
~/isaacsim-venv/bin/python ~/waffle_sim/sim/isaac/import_waffle_4wd.py --urdf ~/waffle_sim/sim_ws/install/waffle_4wd_description_261027_1711/share/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf --package-dir ~/waffle_sim/sim_ws/install/waffle_4wd_description_261027_1711/share/waffle_4wd_description_261027_1711 --out ~/waffle_sim/sim/isaac/usd_rebuild --/crashreporter/url=
```

USD 가져오기 설정: `merge_fixed_joints=False`(센서 프레임 보존), `collision_from_visuals=False`(URDF 충돌 형상 사용), `fix_base=False`(이동 로봇), `allow_self_collision=True`(팔-팔·팔-차체 충돌), 관절 목표 바퀴 = 속도·팔·집게 = 위치(drive type force), 강성·감쇠는 지정하지 않음.

(필수) USD 판정 ②~⑩ · PC — USD를 새 프로세스에서 다시 열어 URDF 값과 비교한다(창 없음, USD 파일은 바꾸지 않음, Physics variant는 저장하지 않는 세션 레이어에서만 `physx`로 고름). 부록 A의 `check_usd_waffle_4wd.py`가 필요하다. 마지막 줄이 `[전체] 성공(조건부 5, 10)`이면 작성자 결과와 같다.
```bash
~/isaacsim-venv/bin/python -I ~/waffle_sim/sim/isaac/check_usd_waffle_4wd.py --urdf ~/waffle_sim/sim_ws/install/waffle_4wd_description_261027_1711/share/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf --usd ~/waffle_sim/sim/isaac/usd/waffle_4wd/waffle_4wd.usda
```
작성자 결과: ② 합성 오류 0개, 찾지 못한 참조 파일 0개, URDF 링크 29개 모두 있음 ③ metersPerUnit 1.0, up Z ④ 질량 있는 링크 14개, 총질량 5.7850283 kg, 최대 오차 링크 질량 상대 3.9e-8·CoM 2.9e-9 m·주관성 모멘트 상대 5.5e-8 ⑤ 가동 관절 11개, 축·한계(최대 2.9e-6°, 집게 6.2e-10 m)·effort·속도 일치, 바퀴는 한계 없음. 물리 고정 관절은 질량 있는 라이다 2개뿐이고 질량 없는 프레임 9개(카메라·scan 등)는 관절 없이 Xform 계층으로 들어간다 ⑥ `gripper_right_joint` 대상 left, 계수 −1 ⑦ 센서 프레임 5개를 포함한 링크 29개 최대 오차 3.9e-9 m, 2.2e-4° ⑧ 551개(Cube 57, Cylinder 4, convexHull Mesh 490) ⑨ 14개 ⑩ `newton:selfCollisionEnabled = 1`.

(선택 · 권장) 충돌 형상 수치 검사 C1~C3 · PC — Isaac Sim 창에서 Colliders를 All로 켜면 이 PC(8 GB GPU)에서 GPU 크래시(`ERROR_DEVICE_LOST`)가 났고, 그 설정이 저장돼 다음 실행이 멈췄다. 화면 대신 이 검사로 본다. 부록 A의 `check_colliders_waffle_4wd.py`가 필요하다. 마지막 줄이 `[전체] 성공`이면 작성자 결과와 같다. 기준: C1 링크마다 충돌 형상과 표시 메시 범위 차이 축마다 ≤ 5 mm, C2 convex hull 부피 ÷ 메시 부피 ≤ 1.01, C3 관절 0 자세에서 관절로 직접 붙지 않은 링크끼리 겹침 0쌍.
```bash
~/isaacsim-venv/bin/python -I ~/waffle_sim/sim/isaac/check_colliders_waffle_4wd.py --usd ~/waffle_sim/sim/isaac/usd/waffle_4wd/waffle_4wd.usda
```
작성자 결과(261027_1711): C1 14링크 모두 성공, C2 메시 489개 모두 닫힘·부피 비 ≤ 1.01, C3 겹침 0쌍. (261027_1530: base_link만 y 1.9 mm — 표시 메시의 바퀴 풀리·축이 벨트 덮개 충돌 상자보다 1.9 mm 더 나옴.) 원본(261021)에서는 C1이 arm_link_5에서 5.5 mm로 실패했고, 원인이 잘못 들어간 집게 나사였어서 261022에서 지웠다. 261027_1711을 처음 만들 때는 벨트 덮개가 옛 충돌 외곽(y ±143.7 mm) 밖으로 나와 C1이 10.3 mm로 실패해서 덮개 충돌 상자 4개를 넣었다.

Isaac Sim에서 Colliders 표시를 쓸 때는 Show By Type > Physics > Colliders를 **Selected**로 두고 링크를 하나씩 고른다. All로 둔 채 종료했다면 USD를 열기 전에 None으로 바꾼다.

조건부의 뜻: 이 importer 버전(3.10.0 이후)은 자기충돌·mimic을 Newton 속성(`newton:selfCollisionEnabled`, `NewtonMimicAPI`)으로만 적고 `PhysxArticulationAPI`·`PhysxMimicJointAPI`를 붙이지 않는다(importer 변경 기록). 관절 속도 한계도 `newton:velocityLimit`만 있고 `physxJoint:maxJointVelocity`는 없다. PhysX가 이 Newton 값들을 실제로 적용하는지는 파일 검사로 알 수 없어 3단계 동작 시험에서 확인한다.

## 6. 현재 상태와 남은 일

| 단계 | 상태 |
|---|---|
| 1 ROS 확인(check_urdf·xacro·colcon·RViz·TF·화면) | 성공 |
| MoveIt 충돌 확인(인접 13쌍만 제외, 기본 자세 충돌 0) | 성공 |
| 2 Isaac Sim 가져오기 | 성공(261027_1711 리비전, importer 오류 0, 판정 ②~⑩ 성공, ⑤·⑩은 조건부, 충돌 형상 C1~C3 성공 — 5-4 참고) |
| 3 작은 동작 시험(중력·접촉·구동, mimic 동작, 자기충돌 비교) | 성공(모의, 261027_1711 재검사: Isaac P1~P8 · MuJoCo M 묶음 · 주행 V 시리즈 판정 항목 모두 통과. 기록 항목 V1 · V1b · V6b · V7은 실패 — 대체된 주행 방식. 계획 · 결과는 `verify_261027_1711.zip`의 `검증계획_261027_1711.md`) |
| 4 ROS↔Isaac 연동(Lyrical 브리지 검증 포함) | 성공(모의, L2 로봇 장면 C1~C5 통과. native Lyrical 브리지(L1)는 CameraInfo 밀림 · LaserScan · TwistStamped 문제로 실패해 영상 외 형식은 Isaac 안 Lyrical rclpy로 발행하는 방식 (a)) |
| 5 Isaac Lab 강화학습 ① 팔 목표 자세 도달 → ② 주행 목표 이동 → ③ 퍽 집기 | 미실시 |
| 6 YOLO(ultralytics, 별도 venv) B/G/R 퍽 검출 | 미실시(맨 마지막) |

2단계 판정 기준(USD를 새 프로세스에서 다시 열고 Physics variant `physx`로 판정, 결과는 5-4): ② 합성 오류 0개, 찾지 못한 참조 파일 0개, URDF 링크 29개(261027_1711) 모두 있음 ③ metersPerUnit 1.0·up Z ④ 질량 있는 링크 14개, 총질량 5.785029 kg(1e-6 kg 이내), 링크별 질량 상대 1e-6·CoM 1e-6 m·주관성 모멘트 상대 1e-5 이내 ⑤ 가동 관절 11개(바퀴 continuous 4, 팔 revolute 5, 집게 prismatic 2), 팔 한계 degree 환산 0.001° 이내·집게 한계 1e-6 m 이내, effort 상대 1e-6·속도 상대 1e-5 이내, 축 방향 내적 1−1e-6 이상 ⑥ `gripper_right_joint` mimic 정보(대상 left, 계수 −1) ⑦ 카메라 3개·라이다 2개 프레임 자세가 URDF로 계산한 관절 0 자세 값과 일치(1e-4 m, 0.01°. 5-2의 TF 값은 1 mm로 반올림한 값이라 판정에 쓰지 않음) ⑧ 충돌 형상 551개(box 57 + cylinder 4 + convex 490) ⑨ 표시 메시 14개 ⑩ articulation 자기충돌 켜짐(Newton 속성으로 판정, PhysX 적용은 3단계).

반드시 알아야 할 주의점:

- **mimic**: 이 importer 버전은 mimic을 `NewtonMimicAPI`로만 기록한다(변경 기록). PhysX에서 오른쪽 집게가 실제로 따라 움직이는지는 3단계에서 확인해야 한다.
- **충돌 근사 차이**: MoveIt은 충돌 메시를 그대로, Isaac(PhysX)은 convex hull로 계산한다. 관절 0 자세에서 Isaac에만 겹침이 생길 수 있다(3단계에서 확인).
- **어깨 범위**: 인수인계 동작 402자세 중 159자세가 어깨 임시 상한(±1.5 rad)을 넘는다(최대 1.912 rad). 한계를 넓히거나 값을 잘라내지 않는다. ③ 퍽 집기 전에 실측 한계 확정 또는 범위 안 새 경로가 필요하다.
- **물성·토크**: 질량·관성은 CAD 추정(출력 부품은 PLA 안쪽 채움 15 %(튜터 확인) + 껍질 0.8 mm(가정), 261027_1711부터), effort(팔 3.8 N·m, 바퀴 1.8 N·m = 모터 2.7 N·m ÷ 증속 1.5)는 11.1 V 순간 stall 값이며 연속 허용값이 아니다. 관절 속도(팔 4.502949 rad/s, 바퀴 10.995574 rad/s)는 무부하 값이다.
- **센서**: 실물 카메라 3개(RealSense 2·Arducam 1), 라이다 3개 중 URDF에는 라이다 2개만 있어 시뮬레이션은 라이다 2개로 가정한다. RealSense 광축은 URDF 기준 아래 35.3°(설계 기억값 약 42°와 차이, 일단 35.3° 사용).
- 자세한 미검증 목록은 `urdf.zip` 안의 `validation.md`, 원본 인수인계 `VALIDATION_PENDING_AND_FAILURES.md`(별도 보관)를 본다.

## 7. 사람마다 결과가 달라질 수 있는 부분

| 구분 | 같은 결과가 나오는 정도 |
|---|---|
| URDF 정적 확인, xacro 비교, TF 값, MoveIt 충돌 판정 | 같은 파일이면 같은 결과(계산이 결정적이다) |
| USD 파일 | 받은 USD를 그대로 쓰면 같다. 다시 만들면 importer 버전이 같아야 같은 구조가 나온다 |
| apt로 설치한 ROS 패키지 | 설치 시점에 따라 버전이 올라갈 수 있다. 1장 버전과 다르면 기록해 둔다 |
| Isaac Sim 물리 재생(3단계 이후) | GPU·드라이버·물리 시간 간격에 따라 작은 수치 차이가 날 수 있다 |
| Isaac Lab 강화학습 | 같은 seed라도 GPU 연산 특성 때문에 학습 곡선이 완전히 같지는 않다. 성능은 GPU VRAM에 따라 환경 수를 다르게 잡아야 할 수 있다 |
| 실행 속도·서브 모니터 멈춤 | 하드웨어(노트북 하이브리드 그래픽 등)에 따라 다르다 |

## 부록 A. 직접 만들 파일 (작성자 PC의 파일 내용 그대로)

아래 각 파일을 제목의 경로에 그대로 저장한다. 코드 블록 안의 내용만 파일에 넣는다(터미널에 붙여 넣지 않는다).

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>waffle_4wd_moveit_config</name>
  <version>0.1.0</version>
  <description>MoveIt 2 설정: Waffle 4WD 팔 충돌 확인용 (SRDF, 기구학, 관절 속도 한계, RViz)</description>
  <maintainer email="dhsckdqja@gmail.com">pa4</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>waffle_4wd_description_261027_1711</exec_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>moveit_kinematics</exec_depend>
  <exec_depend>moveit_planners_ompl</exec_depend>
  <exec_depend>moveit_ros_visualization</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.22)
project(waffle_4wd_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
ament_package()
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/.setup_assistant`

```yaml
moveit_setup_assistant_config:
  urdf:
    package: waffle_4wd_description_261027_1711
    relative_path: urdf/waffle_4wd.urdf.xacro
  srdf:
    relative_path: config/waffle_4wd.srdf
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/config/waffle_4wd.srdf`

```xml
<?xml version="1.0"?>
<!-- Waffle 4WD MoveIt 의미 정보. 충돌 검사 제외 쌍(disable_collisions)은 collisions_updater로 채운다:
     관절로 바로 붙은 쌍(Adjacent)과 관절 0 자세에서 닿아 있는 쌍(Default)·항상 닿는 쌍(Always)만 제외하고,
     무작위 표본에서 한 번도 안 닿았다는 이유(Never)로는 제외하지 않는다(드문 팔-차체 충돌을 놓치지 않기 위해). -->
<robot name="waffle_4wd">
  <!-- 팔 5관절: base_link에서 arm_link_5까지 -->
  <group name="arm">
    <chain base_link="base_link" tip_link="arm_link_5"/>
  </group>
  <!-- 집게: 오른쪽은 URDF에서 왼쪽의 -1 mimic -->
  <group name="gripper">
    <joint name="gripper_left_joint"/>
    <joint name="gripper_right_joint"/>
  </group>
  <group_state name="zero" group="arm">
    <joint name="RMX52_Base_Yaw" value="0"/>
    <joint name="RMX52_Shoulder" value="0"/>
    <joint name="RMX52_Elbow" value="0"/>
    <joint name="RMX52_Wrist_Pitch" value="0"/>
    <joint name="RMX52_Wrist_Roll_Reversed" value="0"/>
  </group_state>
  <end_effector name="gripper_ee" parent_link="arm_link_5" group="gripper" parent_group="arm"/>
  <!-- 충돌 확인은 차체를 고정한 상태로 한다 -->
  <virtual_joint name="virtual_joint" type="fixed" parent_frame="world" child_link="base_footprint"/>
  <!-- 바퀴는 팔 계획 대상이 아니다 -->
  <passive_joint name="Wheel_Revolute_Front_Left"/>
  <passive_joint name="Wheel_Revolute_Front_Right"/>
  <passive_joint name="Wheel_Revolute_Rear_Left"/>
  <passive_joint name="Wheel_Revolute_Rear_Right"/>
  <!-- 충돌 검사 제외: 관절로 바로 붙은 쌍(Adjacent) 13개만. 관절 0 자세에서 닿는 쌍(Default/Always)은 0개였다.
       collisions_updater가 제안한 Never 49쌍(arm_link_2-base_link, arm_link_3-arm_link_5 등 포함)은 넣지 않았다. -->
  <disable_collisions link1="arm_link_1" link2="arm_link_2" reason="Adjacent"/>
  <disable_collisions link1="arm_link_1" link2="base_link" reason="Adjacent"/>
  <disable_collisions link1="arm_link_2" link2="arm_link_3" reason="Adjacent"/>
  <disable_collisions link1="arm_link_3" link2="arm_link_4" reason="Adjacent"/>
  <disable_collisions link1="arm_link_4" link2="arm_link_5" reason="Adjacent"/>
  <disable_collisions link1="arm_link_5" link2="gripper_left_link" reason="Adjacent"/>
  <disable_collisions link1="arm_link_5" link2="gripper_right_link" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="lidar_front" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="lidar_rear" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="wheel_front_left" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="wheel_front_right" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="wheel_rear_left" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="wheel_rear_right" reason="Adjacent"/>
</robot>
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/config/kinematics.yaml`

```yaml
# 팔이 5자유도라 6자유도 자세 IK는 일반적으로 풀리지 않는다. 위치만 맞추는 IK로 둔다.
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: true
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/config/joint_limits.yaml`

```yaml
# URDF(261027_1711)의 속도 한계를 그대로 쓴다(rad/s, m/s): 팔 = XM430-W350 무부하 속도 43 rev/min(11.1 V), 집게 0.01(임시). 가속도 한계는 아직 정하지 않았다.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
joint_limits:
  RMX52_Base_Yaw: {has_velocity_limits: true, max_velocity: 4.502949, has_acceleration_limits: false, max_acceleration: 0.0}
  RMX52_Shoulder: {has_velocity_limits: true, max_velocity: 4.502949, has_acceleration_limits: false, max_acceleration: 0.0}
  RMX52_Elbow: {has_velocity_limits: true, max_velocity: 4.502949, has_acceleration_limits: false, max_acceleration: 0.0}
  RMX52_Wrist_Pitch: {has_velocity_limits: true, max_velocity: 4.502949, has_acceleration_limits: false, max_acceleration: 0.0}
  RMX52_Wrist_Roll_Reversed: {has_velocity_limits: true, max_velocity: 4.502949, has_acceleration_limits: false, max_acceleration: 0.0}
  gripper_left_joint: {has_velocity_limits: true, max_velocity: 0.01, has_acceleration_limits: false, max_acceleration: 0.0}
  gripper_right_joint: {has_velocity_limits: true, max_velocity: 0.01, has_acceleration_limits: false, max_acceleration: 0.0}
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/config/collision_check.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Reference Frame: base_footprint
    - Class: moveit_rviz_plugin/MotionPlanning
      Name: MotionPlanning
      Enabled: true
      Move Group Namespace: ""
      Robot Description: robot_description
      Planning Request:
        Planning Group: arm
        Query Start State: false
        Query Goal State: true
        Interactive Marker Size: 0
        Colliding Link Color: 255; 0; 0
        Goal State Color: 250; 128; 0
        Goal State Alpha: 1
        Joint Violation Color: 255; 0; 255
      Scene Robot:
        Show Robot Visual: true
        Show Robot Collision: false
        Robot Alpha: 0.3
      Planned Path:
        Show Robot Visual: false
        Show Trail: false
  Enabled: true
  Global Options:
    Fixed Frame: base_footprint
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.6
      Focal Point:
        X: 0
        Y: 0
        Z: 0.3
      Pitch: 0.45
      Yaw: 0.8
      Target Frame: base_footprint
Window Geometry:
  Height: 900
  Width: 1500
```

### `~/waffle_sim/sim_ws/src/waffle_4wd_moveit_config/launch/collision_check.launch.py`

```python
"""Waffle 4WD 팔 충돌 확인용 MoveIt 2 실행.

차체를 고정한 상태(virtual_joint fixed)로 move_group과 RViz MotionPlanning을 띄운다.
RViz의 MotionPlanning > Joints 탭에서 목표 자세를 움직이면, 충돌하는 링크가 빨간색으로 표시된다.
궤적 실행(컨트롤러)은 쓰지 않는다.

사용: ros2 launch waffle_4wd_moveit_config collision_check.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('waffle_4wd', package_name='waffle_4wd_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    rviz_file = os.path.join(get_package_share_directory('waffle_4wd_moveit_config'), 'config', 'collision_check.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[moveit_config.robot_description],
            output='screen',
        ),
        # 현재 상태는 모든 관절 0으로 발행한다(충돌 확인은 RViz의 목표 자세로 한다)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[moveit_config.robot_description],
            output='screen',
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[moveit_config.to_dict(), {'allow_trajectory_execution': False}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            output='screen',
        ),
    ])
```

### `~/waffle_sim/sim/tools/check_arm_collision.py`

```python
"""move_group의 /check_state_validity로 팔 자세의 충돌 여부를 확인한다.

사용 (collision_check.launch.py 실행 중):
  python3 check_arm_collision.py                      # 관절 0 자세
  python3 check_arm_collision.py 0 1.2 -1.0 0.5 0      # yaw shoulder elbow pitch roll [rad]
출력: 유효 여부와 충돌한 링크 쌍(접촉 깊이 m).
"""
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

ARM_JOINTS = ['RMX52_Base_Yaw', 'RMX52_Shoulder', 'RMX52_Elbow', 'RMX52_Wrist_Pitch', 'RMX52_Wrist_Roll_Reversed']


def main():
    values = [float(v) for v in sys.argv[1:]] or [0.0] * len(ARM_JOINTS)
    if len(values) != len(ARM_JOINTS):
        sys.exit(f'관절 값 {len(ARM_JOINTS)}개가 필요하다: {ARM_JOINTS}')
    rclpy.init()
    node = rclpy.create_node('check_arm_collision')
    client = node.create_client(GetStateValidity, '/check_state_validity')
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit('/check_state_validity 서비스가 없다. collision_check.launch.py를 먼저 실행한다')
    request = GetStateValidity.Request()
    request.group_name = 'arm'
    request.robot_state = RobotState(joint_state=JointState(name=ARM_JOINTS, position=values), is_diff=True)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    pairs = sorted({(c.contact_body_1, c.contact_body_2, round(c.depth, 4)) for c in result.contacts})
    print(f'자세 {dict(zip(ARM_JOINTS, values))}')
    print(f'유효(충돌 없음): {result.valid}, 충돌 쌍 {len(pairs)}개')
    for a, b, depth in pairs:
        print(f'  {a} <-> {b}  깊이 {depth} m')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### `~/waffle_sim/sim/isaac/import_waffle_4wd.py`

```python
"""Waffle 4WD URDF를 Isaac Sim 6.1 USD로 가져온다(창 없이).

가져오기 설정(2단계에서 합의한 값):
  - merge_fixed_joints=False   : 카메라·라이다 프레임 9개를 합치지 않고 보존한다
  - collision_from_visuals=False: URDF의 충돌 형상(box·cylinder·convex STL)을 그대로 쓴다
  - fix_base=False             : 이동 로봇이므로 바닥에 고정하지 않는다
  - allow_self_collision=True  : 팔-팔, 팔-차체 충돌로 움직임이 막히게 한다
  - 관절 목표: 바퀴 = 속도, 팔·집게 = 위치 (강성·감쇠는 바꾸지 않는다. 3단계에서 정한다)

사용 (PC):
  ~/isaacsim-venv/bin/python import_waffle_4wd.py \
      --urdf <패키지>/urdf/waffle_4wd.urdf --package-dir <패키지 폴더> --out <USD 저장 폴더> --/crashreporter/url=
결과: <USD 저장 폴더>/waffle_4wd/waffle_4wd.usda (같은 이름 폴더가 있으면 importer가 새 이름을 만든다)
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--urdf', required=True, help='waffle_4wd.urdf 경로')
parser.add_argument('--package-dir', required=True, help='URDF 패키지 폴더(package:// 경로 해석용)')
parser.add_argument('--package-name', help='package:// 이름. 주지 않으면 패키지 폴더의 package.xml <name>을 쓴다')
parser.add_argument('--out', required=True, help='USD를 저장할 폴더')
args, _ = parser.parse_known_args()   # 나머지(--/crashreporter/url= 등)는 Kit 설정으로 넘긴다
if not args.package_name:
    import xml.etree.ElementTree as ET
    args.package_name = ET.parse(os.path.join(args.package_dir, 'package.xml')).getroot().findtext('name').strip()

from isaacsim import SimulationApp

simulation_app = SimulationApp({'headless': True})

import isaacsim.core.experimental.utils.app as app_utils

app_utils.enable_extension('isaacsim.asset.importer.urdf')
simulation_app.update()

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig(
    urdf_path=os.path.abspath(args.urdf),
    usd_path=os.path.abspath(args.out),
    merge_fixed_joints=False,
    collision_from_visuals=False,
    allow_self_collision=True,
    ros_package_paths=[{'name': args.package_name, 'path': os.path.abspath(args.package_dir)}],
    fix_base=False,
    joint_drive_type='force',
    joint_target_type={'Wheel_.*': 'velocity', 'RMX52_.*': 'position', 'gripper_.*': 'position'},
)
os.makedirs(config.usd_path, exist_ok=True)
exit_code = 0
try:
    usd_file = URDFImporter(config).import_urdf()
    print(f'[import] USD 생성: {usd_file} (존재: {os.path.isfile(usd_file)})', flush=True)
    if not os.path.isfile(usd_file):
        exit_code = 1
except Exception as error:   # 실패를 성공처럼 끝내지 않는다
    import traceback
    traceback.print_exc()
    print(f'[import] 실패: {error}', flush=True)
    exit_code = 1
finally:
    sys.stdout.flush()
    sys.stderr.flush()
    simulation_app.close()
    os._exit(exit_code)
```

### `~/waffle_sim/sim/isaac/check_usd_waffle_4wd.py`

```python
"""Waffle 4WD USD를 새 프로세스에서 다시 열어 URDF 값과 비교한다(2단계 판정 ②~⑩).

USD 파일은 바꾸지 않는다. Physics variant 선택은 세션 레이어에만 하고 저장하지 않는다.

사용 (PC):
  ~/isaacsim-venv/bin/python -I check_usd_waffle_4wd.py \
      --urdf <패키지>/urdf/waffle_4wd.urdf --usd <USD 폴더>/waffle_4wd/waffle_4wd.usda [--json 결과.json]
종료 코드: 모든 판정이 성공이면 0, 아니면 1
"""
import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

# 판정 허용 오차
TOL_TOTAL_MASS_KG = 1e-6
TOL_LINK_MASS_REL = 1e-6        # 링크 질량 상대 오차(USD는 float32로 저장)
TOL_COM_M = 1e-6
TOL_INERTIA_REL = 1e-5          # 주관성 모멘트 상대 오차
TOL_LIMIT_DEG = 1e-3
TOL_LIMIT_M = 1e-6
TOL_EFFORT_REL = 1e-6
TOL_VELOCITY_REL = 1e-5
TOL_AXIS_DOT = 1e-6             # 1 − (축 내적) 허용값
TOL_POS_M = 1e-4
TOL_ROT_DEG = 0.01

# 기대 개수(URDF에서 다시 세어 함께 비교한다)
EXPECTED = {
    'mass_links': 14,
    'total_mass_kg': 4.220692,  # 261027_1711 리비전(261027_1530 5.785029, 261022_2119까지 4.891194)
    'movable_joints': 11,       # continuous 4 + revolute 5 + prismatic 2
    'collision': {'Cube': 56, 'Cylinder': 4, 'Mesh': 489},   # 261027_1711 리비전(무게추 컵 상자 −1, rail 볼록 64 → 63. 261027_1530 Cube 57 · Mesh 490, 261022_2119까지 Cube 48)
    'visual_meshes': 14,
}
SENSOR_FRAMES = ['camera_optical_frame', 'top_camera_front_optical_frame',
                 'top_camera_rear_optical_frame', 'lidar_front_scan', 'lidar_rear_scan']


# ---------- 수학 도구 ----------
def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_to_matrix(q):
    """Gf.Quat(f|d) → 3×3 회전 행렬."""
    w = q.GetReal()
    x, y, z = q.GetImaginary()
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def homogeneous(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def rotation_angle_deg(ra, rb):
    c = (np.trace(ra.T @ rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def gf_to_numpy(matrix):
    """Gf.Matrix4d(행 벡터 규약) → 열 벡터 규약 4×4."""
    return np.array([[matrix[i][j] for j in range(4)] for i in range(4)]).T


def vec(text, default='0 0 0'):
    return np.array([float(v) for v in (text or default).split()])


# ---------- URDF ----------
def load_urdf(path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall('link'):
        info = {'name': link.get('name'), 'inertial': None, 'visuals': 0, 'collisions': {}}
        inertial = link.find('inertial')
        if inertial is not None:
            origin = inertial.find('origin')
            i = inertial.find('inertia').attrib
            tensor = np.array([
                [float(i['ixx']), float(i['ixy']), float(i['ixz'])],
                [float(i['ixy']), float(i['iyy']), float(i['iyz'])],
                [float(i['ixz']), float(i['iyz']), float(i['izz'])]])
            rot = rpy_to_matrix(vec(origin.get('rpy') if origin is not None else None))
            info['inertial'] = {
                'mass': float(inertial.find('mass').get('value')),
                'com': vec(origin.get('xyz') if origin is not None else None),
                'tensor_link': rot @ tensor @ rot.T,
            }
        info['visuals'] = len(link.findall('visual'))
        for col in link.findall('collision'):
            tag = col.find('geometry')[0].tag
            info['collisions'][tag] = info['collisions'].get(tag, 0) + 1
        links[info['name']] = info
    joints = {}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        limit = joint.find('limit')
        mimic = joint.find('mimic')
        joints[joint.get('name')] = {
            'type': joint.get('type'),
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'xyz': vec(origin.get('xyz') if origin is not None else None),
            'rot': rpy_to_matrix(vec(origin.get('rpy') if origin is not None else None)),
            'axis': vec(joint.find('axis').get('xyz') if joint.find('axis') is not None else None, '1 0 0'),
            'limit': None if limit is None else {k: float(v) for k, v in limit.attrib.items()},
            'mimic': None if mimic is None else {
                'joint': mimic.get('joint'),
                'multiplier': float(mimic.get('multiplier', '1')),
                'offset': float(mimic.get('offset', '0'))},
        }
    children = {j['child'] for j in joints.values()}
    roots = [name for name in links if name not in children]
    assert len(roots) == 1, roots
    # 관절 0 자세에서 root 기준 각 링크 자세와 root에서의 경로
    poses, paths = {roots[0]: np.eye(4)}, {roots[0]: [roots[0]]}
    pending = list(joints.values())
    while pending:
        rest = []
        for j in pending:
            if j['parent'] in poses:
                poses[j['child']] = poses[j['parent']] @ homogeneous(j['rot'], j['xyz'])
                paths[j['child']] = paths[j['parent']] + [j['child']]
            else:
                rest.append(j)
        assert len(rest) < len(pending)
        pending = rest
    return {'root': roots[0], 'links': links, 'joints': joints, 'poses': poses, 'paths': paths}


# ---------- 판정 기록 ----------
class Report:
    def __init__(self):
        self.items = {}

    def add(self, key, ok, detail):
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key]['ok'] = self.items[key]['ok'] and bool(ok)
        self.items[key]['details'].append(('성공 ' if ok else '실패 ') + detail)

    def note(self, key, detail):
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key]['details'].append('참고 ' + detail)

    def condition(self, key, detail):
        """정적 검사로는 확정할 수 없어 동작 시험에서 확인할 조건을 적는다."""
        self.items.setdefault(key, {'ok': True, 'details': []})
        self.items[key].setdefault('conditions', []).append(detail)
        self.items[key]['details'].append('조건 ' + detail)


def api_schemas(prim):
    """합성된 apiSchemas 메타데이터. pxr 단독 환경에는 Newton·PhysX 스키마가 등록돼 있지 않아
    GetAppliedSchemas()에 나오지 않으므로 메타데이터를 직접 읽는다."""
    op = prim.GetMetadata('apiSchemas')
    return [str(t) for t in op.GetAddedOrExplicitItems()] if op else []


def attr(prim, name):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasAuthoredValue() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--usd', required=True)
    parser.add_argument('--variant', default='physx', help='판정에 쓸 Physics variant')
    parser.add_argument('--json', help='판정 결과를 저장할 JSON 경로')
    args = parser.parse_args()

    urdf = load_urdf(args.urdf)
    rep = Report()

    # ② 다시 열기: 합성 오류, 참조 파일, 기본 variant
    stage = Usd.Stage.Open(args.usd, Usd.Stage.LoadAll)
    rep.add('2', stage is not None, f'Usd.Stage.Open 성공 ({args.usd})')
    errors = stage.GetCompositionErrors() if hasattr(stage, 'GetCompositionErrors') else []
    rep.add('2', len(errors) == 0, f'합성 오류 {len(errors)}개')
    for e in errors[:10]:
        rep.note('2', f'합성 오류: {e}')
    _, _, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath(args.usd))
    rep.add('2', len(unresolved) == 0, f'찾지 못한 참조 파일 {len(unresolved)}개 {list(unresolved)[:5]}')
    robot = stage.GetDefaultPrim()
    vset = robot.GetVariantSets().GetVariantSet('Physics')
    default_selection = vset.GetVariantSelection()
    joints_without_variant = sum(1 for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint))
    rep.note('2', f'파일에 저장된 Physics variant 선택: "{default_selection}" (선택지 {vset.GetVariantNames()}), '
                  f'이 상태의 물리 관절 수 {joints_without_variant}')
    # 판정은 세션 레이어에서 variant를 골라서 한다(파일은 바꾸지 않는다)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        vset.SetVariantSelection(args.variant)
    rep.note('2', f'판정용 variant: "{vset.GetVariantSelection()}" (세션 레이어에만 설정, 저장 안 함)')

    # ③ 단위·축
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    rep.add('3', mpu == 1.0, f'metersPerUnit = {mpu}')
    rep.add('3', up == UsdGeom.Tokens.z, f'upAxis = {up}')
    rep.note('3', f'kilogramsPerUnit = {UsdPhysics.GetStageKilogramsPerUnit(stage)}')

    # 링크 prim 찾기: /<robot>/Geometry/<URDF 경로>
    geometry_root = robot.GetPath().AppendChild('Geometry')
    link_prims = {}
    for name, path in urdf['paths'].items():
        prim_path = geometry_root
        for part in path:
            prim_path = prim_path.AppendChild(part)
        prim = stage.GetPrimAtPath(prim_path)
        link_prims[name] = prim if prim.IsValid() else None
    missing = [n for n, p in link_prims.items() if p is None]
    rep.add('2', not missing, f'URDF 링크 {len(urdf["links"])}개 중 USD prim 없음 {len(missing)}개 {missing}')

    # ④ 질량·CoM·주관성 모멘트
    mass_prims = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.MassAPI)]
    urdf_mass_links = [n for n, l in urdf['links'].items() if l['inertial'] and l['inertial']['mass'] > 0]
    rep.add('4', len(mass_prims) == EXPECTED['mass_links'] == len(urdf_mass_links),
            f'질량 있는 링크: USD {len(mass_prims)}개, URDF {len(urdf_mass_links)}개, 기대 {EXPECTED["mass_links"]}개')
    total = 0.0
    worst = {'mass_rel': 0.0, 'com_m': 0.0, 'inertia_rel': 0.0, 'tensor_rel': 0.0}
    for name in urdf_mass_links:
        prim = link_prims.get(name)
        ref = urdf['links'][name]['inertial']
        if prim is None or not prim.HasAPI(UsdPhysics.MassAPI):
            rep.add('4', False, f'{name}: USD에 MassAPI 없음')
            continue
        mass = attr(prim, 'physics:mass')
        com = np.array(attr(prim, 'physics:centerOfMass'))
        diag = np.array(attr(prim, 'physics:diagonalInertia'))
        axes = quat_to_matrix(attr(prim, 'physics:principalAxes'))
        total += mass
        mass_rel = abs(mass - ref['mass']) / ref['mass']
        com_err = float(np.max(np.abs(com - ref['com'])))
        eig_ref = np.sort(np.linalg.eigvalsh(ref['tensor_link']))
        inertia_rel = float(np.max(np.abs(np.sort(diag) - eig_ref) / eig_ref))
        tensor_usd = axes @ np.diag(diag) @ axes.T
        tensor_rel = float(np.max(np.abs(tensor_usd - ref['tensor_link'])) / np.max(eig_ref))
        worst = {k: max(worst[k], v) for k, v in
                 zip(worst, (mass_rel, com_err, inertia_rel, tensor_rel))}
        ok = mass_rel <= TOL_LINK_MASS_REL and com_err <= TOL_COM_M and inertia_rel <= TOL_INERTIA_REL
        if not ok:
            rep.add('4', False, f'{name}: 질량 상대오차 {mass_rel:.2e}, CoM {com_err:.2e} m, 주관성 상대오차 {inertia_rel:.2e}')
    total_urdf = sum(urdf['links'][n]['inertial']['mass'] for n in urdf_mass_links)
    rep.add('4', abs(total - EXPECTED['total_mass_kg']) <= TOL_TOTAL_MASS_KG,
            f'총질량 USD {total:.7f} kg, URDF {total_urdf:.7f} kg, 기대 {EXPECTED["total_mass_kg"]} kg')
    rep.add('4', worst['mass_rel'] <= TOL_LINK_MASS_REL, f'링크 질량 최대 상대오차 {worst["mass_rel"]:.2e} (허용 {TOL_LINK_MASS_REL:g})')
    rep.add('4', worst['com_m'] <= TOL_COM_M, f'CoM 최대 오차 {worst["com_m"]:.2e} m (허용 {TOL_COM_M:g})')
    rep.add('4', worst['inertia_rel'] <= TOL_INERTIA_REL,
            f'주관성 모멘트 최대 상대오차 {worst["inertia_rel"]:.2e} (허용 {TOL_INERTIA_REL:g})')
    rep.note('4', f'링크 좌표계 관성 텐서(주축 회전 포함) 최대 오차 / 최대 주관성 = {worst["tensor_rel"]:.2e}')

    # ⑤ 관절
    joint_prims = {p.GetName(): p for p in Usd.PrimRange(robot) if p.IsA(UsdPhysics.Joint)}
    movable_urdf = {n: j for n, j in urdf['joints'].items() if j['type'] != 'fixed'}
    movable_usd = {n: p for n, p in joint_prims.items() if not p.IsA(UsdPhysics.FixedJoint)}
    rep.add('5', len(movable_usd) == len(movable_urdf) == EXPECTED['movable_joints'],
            f'가동 관절: USD {len(movable_usd)}개, URDF {len(movable_urdf)}개, 기대 {EXPECTED["movable_joints"]}개')
    # 질량 없는 프레임(카메라·scan 등)은 강체가 아니므로 물리 고정 관절 없이 Xform 계층으로만 들어간다.
    # 물리 고정 관절은 양쪽 링크가 모두 질량을 가진 URDF 고정 관절만 기대한다(프레임 자세는 ⑦에서 본다).
    def has_mass(link):
        inertial = urdf['links'][link]['inertial']
        return bool(inertial and inertial['mass'] > 0)
    fixed_rigid_urdf = sorted(n for n, j in urdf['joints'].items()
                              if j['type'] == 'fixed' and has_mass(j['parent']) and has_mass(j['child']))
    fixed_usd = sorted(n for n, p in joint_prims.items() if p.IsA(UsdPhysics.FixedJoint))
    rep.add('5', fixed_usd == fixed_rigid_urdf,
            f'물리 고정 관절: USD {fixed_usd}, URDF(양쪽 질량 있음) {fixed_rigid_urdf}')
    frame_only = sorted(n for n, j in urdf['joints'].items() if j['type'] == 'fixed' and n not in fixed_rigid_urdf)
    rep.note('5', f'질량 없는 프레임으로 이어진 URDF 고정 관절 {len(frame_only)}개는 물리 관절 없이 Xform 계층: {frame_only}')
    type_map = {'revolute': UsdPhysics.RevoluteJoint, 'continuous': UsdPhysics.RevoluteJoint,
                'prismatic': UsdPhysics.PrismaticJoint}
    unit_axes = {'X': np.array([1.0, 0, 0]), 'Y': np.array([0, 1.0, 0]), 'Z': np.array([0, 0, 1.0])}
    physx_velocity_authored = 0
    for name, ref in movable_urdf.items():
        prim = movable_usd.get(name)
        if prim is None:
            rep.add('5', False, f'{name}: USD 관절 없음')
            continue
        problems = []
        if not prim.IsA(type_map[ref['type']]):
            problems.append(f'형식 {prim.GetTypeName()}')
        body0 = prim.GetRelationship('physics:body0').GetTargets()
        body1 = prim.GetRelationship('physics:body1').GetTargets()
        if [b.name for b in body0] != [ref['parent']] or [b.name for b in body1] != [ref['child']]:
            problems.append(f'연결 {body0}→{body1}')
        pos0 = np.array(attr(prim, 'physics:localPos0'))
        rot0 = quat_to_matrix(attr(prim, 'physics:localRot0'))
        axis_usd = rot0 @ unit_axes[attr(prim, 'physics:axis')]
        axis_ref = ref['rot'] @ (ref['axis'] / np.linalg.norm(ref['axis']))
        axis_dot = float(axis_usd @ axis_ref)
        if 1.0 - axis_dot > TOL_AXIS_DOT:
            problems.append(f'축 내적 {axis_dot:.8f}')
        if np.max(np.abs(pos0 - ref['xyz'])) > TOL_POS_M:
            problems.append(f'관절 위치 {pos0} ≠ {ref["xyz"]}')
        lower, upper = attr(prim, 'physics:lowerLimit'), attr(prim, 'physics:upperLimit')
        kind = 'angular' if ref['type'] != 'prismatic' else 'linear'
        limit_text = ''
        if ref['type'] == 'continuous':
            unlimited = (lower is None or lower <= -1e30) and (upper is None or upper >= 1e30)
            if not unlimited:
                problems.append(f'continuous인데 한계 {lower}~{upper}')
            limit_text = f'한계 없음(USD {lower}~{upper})'
        elif ref['type'] == 'revolute':
            lo, hi = math.degrees(ref['limit']['lower']), math.degrees(ref['limit']['upper'])
            err = max(abs(lower - lo), abs(upper - hi))
            if err > TOL_LIMIT_DEG:
                problems.append(f'한계 오차 {err:.2e}°')
            limit_text = f'한계 {lower:.4f}~{upper:.4f}° (오차 {err:.1e}°)'
        else:
            lo, hi = ref['limit']['lower'], ref['limit']['upper']
            err = max(abs(lower - lo), abs(upper - hi))
            if err > TOL_LIMIT_M:
                problems.append(f'한계 오차 {err:.2e} m')
            limit_text = f'한계 {lower:.7f}~{upper:.7f} m (오차 {err:.1e} m)'
        effort = attr(prim, f'drive:{kind}:physics:maxForce')
        if effort is None or abs(effort - ref['limit']['effort']) > TOL_EFFORT_REL * ref['limit']['effort']:
            problems.append(f'effort {effort} ≠ {ref["limit"]["effort"]}')
        velocity = attr(prim, 'newton:velocityLimit')
        ref_velocity = ref['limit']['velocity']
        ref_velocity_usd = math.degrees(ref_velocity) if kind == 'angular' else ref_velocity
        if velocity is None or abs(velocity - ref_velocity_usd) > TOL_VELOCITY_REL * ref_velocity_usd:
            problems.append(f'newton:velocityLimit {velocity} ≠ {ref_velocity_usd}')
        physx_velocity = attr(prim, 'physxJoint:maxJointVelocity')
        if physx_velocity is not None:
            physx_velocity_authored += 1
        drive_type = attr(prim, f'drive:{kind}:physics:type')
        detail = (f'{name}: {ref["type"]}, 축 내적 {axis_dot:.8f}, {limit_text}, effort {effort}, '
                  f'속도 newton {velocity} ({"deg/s" if kind == "angular" else "m/s"}), '
                  f'physxJoint:maxJointVelocity {physx_velocity}, drive {drive_type}')
        rep.add('5', not problems, detail + (f' — 문제: {problems}' if problems else ''))
    rep.note('5', f'PhysX 관절 속도 한계(physxJoint:maxJointVelocity)가 적힌 관절 {physx_velocity_authored}개 / {len(movable_urdf)}개')
    if physx_velocity_authored < len(movable_urdf):
        rep.condition('5', '속도 한계는 newton:velocityLimit으로만 판정했다. PhysX가 이 값을 적용하는지는 동작 시험에서 확인한다')

    # ⑥ mimic
    for name, ref in urdf['joints'].items():
        if not ref['mimic']:
            continue
        prim = joint_prims.get(name)
        targets = prim.GetRelationship('newton:mimicJoint').GetTargets() if prim else []
        coef1 = attr(prim, 'newton:mimicCoef1') if prim else None
        coef0 = attr(prim, 'newton:mimicCoef0') if prim else None
        ok = ([t.name for t in targets] == [ref['mimic']['joint']] and coef1 == ref['mimic']['multiplier']
              and (coef0 or 0.0) == ref['mimic']['offset'])
        schemas = api_schemas(prim) if prim else []
        rep.add('6', ok, f'{name}: 대상 {[str(t) for t in targets]}, 계수 {coef1}, 오프셋 {coef0} '
                         f'(URDF 대상 {ref["mimic"]["joint"]}, 계수 {ref["mimic"]["multiplier"]}, 오프셋 {ref["mimic"]["offset"]})')
        rep.note('6', f'{name} 적용 스키마 {schemas}. PhysX mimic(PhysxMimicJointAPI) 적용 여부: '
                      f'{any("PhysxMimicJointAPI" in s for s in schemas)}')

    # ⑦ 프레임 자세(관절 0 자세, root 링크 기준)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    root_world = gf_to_numpy(cache.GetLocalToWorldTransform(link_prims[urdf['root']]))
    worst_pos, worst_rot = 0.0, 0.0
    for name, prim in link_prims.items():
        if prim is None:
            continue
        rel = np.linalg.inv(root_world) @ gf_to_numpy(cache.GetLocalToWorldTransform(prim))
        ref = urdf['poses'][name]
        scale = np.linalg.norm(rel[:3, :3], axis=0)
        rot = rel[:3, :3] / scale
        pos_err = float(np.max(np.abs(rel[:3, 3] - ref[:3, 3])))
        rot_err = rotation_angle_deg(rot, ref[:3, :3])
        worst_pos, worst_rot = max(worst_pos, pos_err), max(worst_rot, rot_err)
        if name in SENSOR_FRAMES:
            rep.add('7', pos_err <= TOL_POS_M and rot_err <= TOL_ROT_DEG,
                    f'{name}: 위치 {np.round(rel[:3, 3], 6).tolist()} m, 위치 오차 {pos_err:.1e} m, 회전 오차 {rot_err:.1e}°')
        elif pos_err > TOL_POS_M or rot_err > TOL_ROT_DEG:
            rep.add('7', False, f'{name}(센서 외 링크): 위치 오차 {pos_err:.1e} m, 회전 오차 {rot_err:.1e}°')
        if np.max(np.abs(scale - 1.0)) > 1e-6:
            rep.note('7', f'{name}: 배율 {scale}')
    rep.note('7', f'링크 {len(link_prims)}개 전체 최대 오차: 위치 {worst_pos:.1e} m, 회전 {worst_rot:.1e}°')

    # ⑧ 충돌 형상, ⑨ 표시 메시 (인스턴스 내부까지 센다)
    collision = {}
    visual_mesh_prims = 0
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            key = prim.GetTypeName()
            if key == 'Mesh':
                key = f'Mesh({attr(prim, "physics:approximation")})'
            collision[key] = collision.get(key, 0) + 1
        elif prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputePurpose() != UsdGeom.Tokens.guide:
            visual_mesh_prims += 1
    urdf_collision = {}
    for link in urdf['links'].values():
        for tag, n in link['collisions'].items():
            urdf_collision[tag] = urdf_collision.get(tag, 0) + n
    usd_counts = {'Cube': collision.get('Cube', 0), 'Cylinder': collision.get('Cylinder', 0),
                  'Mesh': sum(v for k, v in collision.items() if k.startswith('Mesh'))}
    rep.add('8', usd_counts == EXPECTED['collision'],
            f'USD 충돌 형상 {collision} 합계 {sum(collision.values())}, 기대 {EXPECTED["collision"]}, URDF {urdf_collision}')
    rep.add('8', collision.get('Mesh(convexHull)', 0) == EXPECTED['collision']['Mesh'],
            f'convexHull 근사 메시 {collision.get("Mesh(convexHull)", 0)}개')
    # URDF visual 하나는 링크 아래 instanceable Xform(이름 = 링크 이름) 하나가 된다
    visual_groups = [p for p in Usd.PrimRange(robot) if p.IsInstance() and p.GetName() in urdf['links']]
    urdf_visuals = sum(l['visuals'] for l in urdf['links'].values())
    rep.add('9', len(visual_groups) == urdf_visuals == EXPECTED['visual_meshes'],
            f'표시 메시 묶음 USD {len(visual_groups)}개, URDF visual {urdf_visuals}개, 기대 {EXPECTED["visual_meshes"]}개')
    rep.note('9', f'표시용 Mesh prim(guide 제외, 하위 메시 포함) {visual_mesh_prims}개')

    # ⑩ 자기충돌
    roots = [p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    rep.add('10', len(roots) == 1, f'ArticulationRootAPI prim {len(roots)}개 {[str(p.GetPath()) for p in roots]}')
    for prim in roots:
        newton = attr(prim, 'newton:selfCollisionEnabled')
        physx = attr(prim, 'physxArticulation:enabledSelfCollisions')
        physx_api = 'PhysxArticulationAPI' in api_schemas(prim)
        rep.add('10', newton is True or physx is True,
                f'{prim.GetName()}: newton:selfCollisionEnabled = {newton}, '
                f'physxArticulation:enabledSelfCollisions = {physx} (PhysxArticulationAPI 적용 {physx_api})')
        if physx is None:
            rep.condition('10', '자기충돌은 newton:selfCollisionEnabled로만 판정했다. PhysX가 이 값을 적용하는지는 동작 시험에서 확인한다')
    filtered = sum(1 for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.FilteredPairsAPI))
    rep.note('10', f'FilteredPairsAPI(충돌 제외 쌍) prim {filtered}개')

    # 출력
    all_ok = True
    for key in sorted(rep.items, key=int):
        item = rep.items[key]
        all_ok = all_ok and item['ok']
        status = '실패' if not item['ok'] else ('조건부 성공' if item.get('conditions') else '성공')
        print(f'[{key}] {status}')
        for line in item['details']:
            print(f'    {line}')
    conditional = [k for k, v in rep.items.items() if v['ok'] and v.get('conditions')]
    print(f'[전체] {"실패 있음" if not all_ok else ("성공(조건부 " + ", ".join(sorted(conditional, key=int)) + ")" if conditional else "성공")}')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(rep.items, f, ensure_ascii=False, indent=2)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
```

### `~/waffle_sim/sim/isaac/check_colliders_waffle_4wd.py`

```python
"""Waffle 4WD USD 충돌 형상을 수치로 확인한다(Colliders 화면 확인 대신).

C1 링크마다 충돌 형상 범위와 표시 메시 범위의 차이(축마다, m)
C2 충돌 메시마다 convex hull 부피 / 원래 메시 부피 (PhysX는 convexHull 근사를 쓴다)
C3 관절 0 자세에서 관절로 직접 붙지 않은 링크끼리 충돌 형상(볼록 형상 기준)이 겹치는 쌍

USD 파일은 바꾸지 않는다(Physics variant는 세션 레이어에서만 고른다).
겹침 판정: 두 볼록 형상의 교집합에 들어가는 가장 큰 구의 반지름(선형계획)으로 잰다.

사용 (PC):
  ~/isaacsim-venv/bin/python -I check_colliders_waffle_4wd.py --usd <...>/waffle_4wd.usda [--json 결과.json]
"""
import argparse
import itertools
import json
import sys

import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

TOL_C1_M = 0.005
TOL_C2_RATIO = 1.01
TOL_C3_OVERLAP_M = 1e-6     # 교집합 내접 구 반지름이 이보다 크면 겹침
CYLINDER_SIDES = 64


def world_points(prim, cache):
    """충돌 형상의 꼭짓점(월드 좌표, m)."""
    m = np.array(cache.GetLocalToWorldTransform(prim)).T   # 열 벡터 규약
    if prim.IsA(UsdGeom.Cube):
        h = prim.GetAttribute('size').Get() / 2.0
        local = np.array(list(itertools.product([-h, h], repeat=3)))
    elif prim.IsA(UsdGeom.Cylinder):
        r = prim.GetAttribute('radius').Get()
        hh = prim.GetAttribute('height').Get() / 2.0
        axis = prim.GetAttribute('axis').Get()
        t = np.linspace(0, 2 * np.pi, CYLINDER_SIDES, endpoint=False)
        ring = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
        pts = [np.insert(ring, 2, z, axis=1) for z in (-hh, hh)]   # z축 원기둥
        local = np.concatenate(pts)
        order = {'X': [2, 0, 1], 'Y': [1, 2, 0], 'Z': [0, 1, 2]}[axis]
        local = local[:, order]
    elif prim.IsA(UsdGeom.Mesh):
        local = np.array(prim.GetAttribute('points').Get(), dtype=float)
    else:
        raise ValueError(f'모르는 형상 {prim.GetTypeName()} {prim.GetPath()}')
    return (m[:3, :3] @ local.T).T + m[:3, 3]


def mesh_volume_ratio(prim):
    pts = np.array(prim.GetAttribute('points').Get(), dtype=float)
    counts = np.array(prim.GetAttribute('faceVertexCounts').Get())
    idx = np.array(prim.GetAttribute('faceVertexIndices').Get())
    faces, k = [], 0
    for c in counts:                         # 다각형 면은 부채꼴로 삼각형 분할
        poly = idx[k:k + c]
        faces += [[poly[0], poly[i], poly[i + 1]] for i in range(1, c - 1)]
        k += c
    tm = trimesh.Trimesh(vertices=pts, faces=np.array(faces), process=True)
    hull_volume = ConvexHull(pts).volume
    return hull_volume / abs(tm.volume), tm.is_watertight, hull_volume


def overlap_radius(points_a, points_b):
    """두 볼록 형상 교집합의 내접 구 반지름. 교집합이 없으면 음수 또는 None."""
    rows, rhs = [], []
    for pts in (points_a, points_b):
        eq = ConvexHull(pts).equations          # n·x + d <= 0
        normal = eq[:, :3]
        norm = np.linalg.norm(normal, axis=1)
        rows.append(np.hstack([normal, norm[:, None]]))   # n·x + |n| r <= -d
        rhs.append(-eq[:, 3])
    res = linprog(c=[0, 0, 0, -1], A_ub=np.vstack(rows), b_ub=np.concatenate(rhs),
                  bounds=[(None, None)] * 3 + [(None, 1.0)], method='highs')
    return res.x[3] if res.status == 0 else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--usd', required=True)
    parser.add_argument('--json')
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd)
    robot = stage.GetDefaultPrim()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        robot.GetVariantSets().GetVariantSet('Physics').SetVariantSelection('physx')
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    bodies = {p.GetName(): p for p in Usd.PrimRange(robot) if p.HasAPI(UsdPhysics.RigidBodyAPI)}

    def owner(prim):
        while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim = prim.GetParent()
        return prim.GetName() if prim else None

    shapes = []   # (링크, prim, 월드 꼭짓점)
    for prim in Usd.PrimRange(robot, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            shapes.append((owner(prim), prim, world_points(prim, cache)))
    result = {'shapes': len(shapes)}
    print(f'충돌 형상 {len(shapes)}개, 강체 링크 {len(bodies)}개')

    # C1: 링크별 범위 비교
    c1, c1_ok = {}, True
    for name, body in bodies.items():
        pts = np.concatenate([s[2] for s in shapes if s[0] == name]) if any(s[0] == name for s in shapes) else None
        visual = body.GetChild(name)          # 표시 메시 묶음 = 링크 아래 같은 이름의 인스턴스
        if pts is None or not visual.IsValid():
            c1[name] = {'note': f'충돌 {pts is not None}, 표시 {visual.IsValid()}'}
            continue
        vr = bbox.ComputeWorldBound(visual).ComputeAlignedRange()
        vmin, vmax = np.array(vr.GetMin()), np.array(vr.GetMax())
        cmin, cmax = pts.min(0), pts.max(0)
        diff = np.maximum(np.abs(cmin - vmin), np.abs(cmax - vmax))
        ok = bool(np.all(diff <= TOL_C1_M))
        c1_ok &= ok
        c1[name] = {'ok': ok, 'max_diff_mm_xyz': (diff * 1000).round(2).tolist(),
                    'collision_min_max_m': [cmin.round(4).tolist(), cmax.round(4).tolist()],
                    'visual_min_max_m': [vmin.round(4).tolist(), vmax.round(4).tolist()]}
    print(f'[C1] {"성공" if c1_ok else "실패"} (축마다 차이 ≤ {TOL_C1_M * 1000:g} mm)')
    for name, v in c1.items():
        print(f'    {name:20s} {v}')
    result['C1'] = {'ok': c1_ok, 'links': c1}

    # C2: convex hull 부피 비
    ratios, not_watertight, worst = [], 0, None
    for name, prim, _ in shapes:
        if not prim.IsA(UsdGeom.Mesh):
            continue
        ratio, watertight, _ = mesh_volume_ratio(prim)
        if not watertight:
            not_watertight += 1
            continue
        ratios.append(ratio)
        if worst is None or ratio > worst[0]:
            worst = (ratio, str(prim.GetPath()))
    ratios = np.array(ratios)
    c2_ok = bool(len(ratios) and ratios.max() <= TOL_C2_RATIO and not_watertight == 0)
    print(f'[C2] {"성공" if c2_ok else "실패"} (hull/메시 부피 ≤ {TOL_C2_RATIO})')
    print(f'    닫힌 메시 {len(ratios)}개, 닫히지 않아 부피를 못 잰 메시 {not_watertight}개')
    if len(ratios):
        print(f'    부피 비 최소 {ratios.min():.6f}, 중앙값 {np.median(ratios):.6f}, 최대 {ratios.max():.6f} ({worst[1]})')
        print(f'    1.01 초과 {int((ratios > TOL_C2_RATIO).sum())}개')
    result['C2'] = {'ok': c2_ok, 'measured': int(len(ratios)), 'not_watertight': not_watertight,
                    'min': float(ratios.min()) if len(ratios) else None,
                    'max': float(ratios.max()) if len(ratios) else None, 'worst': worst}

    # C3: 관절로 직접 붙지 않은 링크 쌍의 겹침
    adjacent, collision_enabled_joints = set(), []
    for prim in Usd.PrimRange(robot):
        if prim.IsA(UsdPhysics.Joint):
            b0 = prim.GetRelationship('physics:body0').GetTargets()
            b1 = prim.GetRelationship('physics:body1').GetTargets()
            if b0 and b1:
                adjacent.add(frozenset((b0[0].name, b1[0].name)))
            enabled = prim.GetAttribute('physics:collisionEnabled')
            if enabled and enabled.HasAuthoredValue() and enabled.Get():
                collision_enabled_joints.append(prim.GetName())
    print(f'[C3] 관절로 붙은 링크 쌍 {len(adjacent)}개(PhysX가 충돌을 빼는 쌍), '
          f'관절 충돌을 켠(collisionEnabled=true) 관절 {collision_enabled_joints}')
    boxes = [(s[2].min(0), s[2].max(0)) for s in shapes]
    tested, overlaps, nearest = 0, [], []
    for i, j in itertools.combinations(range(len(shapes)), 2):
        a, b = shapes[i], shapes[j]
        if a[0] == b[0] or frozenset((a[0], b[0])) in adjacent:
            continue
        if np.any(boxes[i][1] < boxes[j][0]) or np.any(boxes[j][1] < boxes[i][0]):
            continue
        tested += 1
        r = overlap_radius(a[2], b[2])
        if r is not None and r > TOL_C3_OVERLAP_M:
            overlaps.append((a[0], b[0], str(a[1].GetPath().name), str(b[1].GetPath().name), r))
        elif r is not None:
            nearest.append((a[0], b[0], r))
    pairs = sorted({(o[0], o[1]) for o in overlaps})
    c3_ok = not overlaps
    print(f'[C3] {"성공" if c3_ok else "실패"} (범위가 겹쳐 정밀 검사한 형상 쌍 {tested}개, 겹친 형상 쌍 {len(overlaps)}개, 링크 쌍 {len(pairs)}개)')
    for o in sorted(overlaps, key=lambda x: -x[4])[:20]:
        print(f'    {o[0]} – {o[1]} ({o[2]} / {o[3]}): 내접 구 반지름 {o[4] * 1000:.3f} mm')
    for n in sorted(nearest, key=lambda x: -x[2]):
        # 음수 값은 겹치지 않았다는 뜻이다(두 형상의 면을 그만큼 밀어야 교집합이 생긴다). 거리 자체는 아니다.
        print(f'    겹치지 않음: {n[0]} – {n[1]} (선형계획 값 {n[2] * 1000:.3f} mm)')
    result['C3'] = {'ok': c3_ok, 'tested_shape_pairs': tested, 'overlap_link_pairs': pairs,
                    'overlaps': [list(o) for o in overlaps], 'not_overlapping_tested': [list(n) for n in nearest]}

    all_ok = c1_ok and c2_ok and c3_ok
    print(f'[전체] {"성공" if all_ok else "실패 있음"}')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(result, f, ensure_ascii=False, indent=2, default=str)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
```
