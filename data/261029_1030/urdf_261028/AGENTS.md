# AGENTS.md — waffle_4wd 로봇 공유본 (AI 도구용 작업 메모)

Claude Code (`CLAUDE.md`), Codex (`AGENTS.md`), Gemini CLI (`GEMINI.md`) 가 같은 내용을 읽도록 본문은 이 파일 하나에 둔다
(`CLAUDE.md` · `GEMINI.md` 는 `@AGENTS.md` 로 가져옴). **AI 도구는 이 폴더 (`urdf_261028/`) 에서 열 것** — 하위 폴더에서 열면 도구에 따라 이 파일을 못 읽는다.
마지막 갱신 2026-10-29. 사람용 설명은 각 폴더 `README.md`.

## 폴더
| 폴더 | 내용 |
|---|---|
| `waffle_4wd_description_261028/` | ROS 2 (Lyrical) 패키지: URDF · xacro · 메시, RViz, Gazebo Sim 10 (`launch/gazebo.launch.py`), 센서 값 `config/sensors.yaml` |
| `isaac/` | Isaac Sim 6.1 / Isaac Lab 3.0: USD 2개 (가벼운 · 정밀 충돌), 물리값 `robot_params.json`, 로봇 · 센서 설정 `waffle_4wd_isaac/`, `run.sh` |

## 틀리기 쉬운 사실 (먼저 읽을 것)
1. **바퀴 간격 = 0.317 m** — 좌우 바퀴 **폭 가운데** 사이 (바퀴 메시 y ±0.1585). URDF 바퀴 **관절 원점** y ±0.1546 (간격 0.3092) 은 폭 가운데보다 3.9 mm 안쪽이라
   `wheel_separation` 으로 쓰면 안 된다. 바퀴 반지름 0.033 m. 휠베이스 (앞뒤 바퀴 축) 0.160 m. 실물 간격도 URDF 와 같음 (팀원 확인).
2. **구동 = 벨트 증속 1 : 1.5** (모터 36T → 바퀴 24T, 실물 확인). 바퀴 한계 10.996 rad/s · 1.8 N·m (URDF). 팀 펌웨어 `config.h` 의 `WHEEL_GEAR` 는
   아직 1.0 (직결) 이고 1.5 로 고칠 예정 — 그 전 실물 odom 은 실제의 2/3, 속도는 명령의 1.5 배. 팀 저장소는 사람이 직접 고친다.
3. **4륜 스키드라 제자리 회전이 명령보다 덜 돈다** (보정에 필요한 배수 시뮬값: Gazebo 약 1.22, Isaac 약 1.42 — 실물은 따로 재야 함). 보정은 diff_drive_controller
   `wheel_separation_multiplier` (실물 측정값). `wheel_separation` 자체를 미끄럼 보정용으로 바꾸지 말 것. 팀 EKF 는 회전 속도를 IMU 에서만 받는다.
4. **손가락 2개에는 같은 값을 명령** — `gripper_left_joint` 축 +y, `gripper_right_joint` 축 −y 라 같은 값이 대칭으로 움직인다. mimic 없음.
   범위 −0.011 (닫힘, 손가락 간격 20 mm) ~ 0.020 m (열림, 82 mm). 반대 부호를 주면 집게가 옆으로 밀린다.
5. **관절 이름은 팀 Lv2 (origin/dev) 와 같다**: 바퀴 `wheel_{left,right}_{front,rear}_joint`, 팔 `joint1~5` (joint5 = 손목 롤),
   `gripper_roll` (그리퍼 개폐 모터 각, 팀 펌웨어 칸 5), 손가락 `gripper_left/right_joint` (직선). 루트 `base_link` (지면 위 6.75 mm), `imu_link` = 두 OpenCR IMU 가운데.
6. **D435 깊이는 받은 값 그대로 URDF optical frame 기준** (2026-10-29 부터). optical frame 은 D435 몸체 메시 앞면 0.2 ~ 0.6 mm 안쪽 (실물과 같음).
   - Gazebo: 깊이 센서가 몸체 안에서 막혀 8 mm 앞에 두고, `scripts/camera_info_fix.py` 가 깊이 영상 +8 mm, 점구름 축 (Gazebo 센서 축 → optical) + 8 mm 로 고쳐 원래 토픽으로 낸다.
   - Isaac: 센서를 frame 그대로 둔다 (근거리 자름 0.105 m 로 몸체가 안 보임).
   - **실물 데이터에 8 mm 를 더하지 말 것.** 수 MB 영상 · 점구름은 reliable QoS 로 구독할 것 (best-effort 는 조각 손실로 주기가 반 이하).
7. LDS-03 ×2 는 상판 밑에 뒤집어 (스캔 높이 지면 위 0.0823 m, x ±0.1089 는 실측 전 임시). 차체에 가려 앞 139.5°, 뒤 195.3° 를 못 본다. TFmini 는 차체 앞 (0.1403, 0, 0.0348).
8. Isaac 시작 자세 `pose('drive' | 'stow' | 'zero')` (기본 drive). 가벼운 판에서 stow ↔ drive 는 zero 를 거칠 것 (바로 가면 팔이 D435 기둥 쪽 상자에 걸림).

## 값의 출처 (`isaac/robot_params.json` 에 근거 · 상태가 함께 있음)
| 값 | 근거 |
|---|---|
| 팔 강성 18.88 N·m/rad · 감쇠 0.851 | 팀 펌웨어 위치 P 800 · I 0 환산. 시뮬에 I 게인 없음 → 실물처럼 처짐 |
| 바퀴 감쇠 0.2133 · armature 0.00222 | W210 기본 Velocity P 100 모터 축 환산 / 1.5² (벨트) |
| 손가락 힘 13 N | 펌웨어 쥐는 힘 33 tick 환산, 기구 효율 0.5 는 추정 |
| 마찰 1.0 | 미측정 (Gazebo 와 같은 값) |
USD 안의 관절 값은 도 (deg) 단위로 저장된다 (강성 · 감쇠 × π/180). Isaac Lab 설정은 `robot_params.json` 으로 USD 값을 덮어쓴다.

## 파일을 바꿀 때
- 생성된 파일은 직접 고치지 말 것: ROS 패키지 `urdf/` · `meshes/` · `config/gz_bridge.yaml` · `config/camera_info_fix.yaml`, Isaac `urdf/` · `usd/` · `results/`.
- ROS 패키지: 센서 · 구동 값은 `config/sensors.yaml` → `python3 tools/make_revision.py` (받은 원본 `~/Downloads/urdf_261027_1711` 과 `arm_bench` URDF 가 있는 PC 에서만 됨.
  없으면 원작업자에게 요청).
- Isaac: 물리값만 바꾸면 `robot_params.json` 수정 → `./run.sh convert` (USD 에도 반영) → `./run.sh check`.
  ROS 패키지 xacro 가 바뀌었으면 먼저 `python3 tools/make_isaac_urdf.py` (ROS 2 xacro 필요).
- 순서: `sensors.yaml` / 생성기 → `make_revision.py` → `make_isaac_urdf.py` → `run.sh convert` → `run.sh check` → Gazebo 시험 → zip.

## 하지 말 것
- 실물 깊이 · 점구름에 8 mm 더하기. 손가락에 반대 부호 명령. `wheel_separation` 을 0.3092 로 쓰기.
- 시뮬 시험에서 ROS 도메인 0 · 서브넷 탐색 그대로 `/cmd_vel` 보내기 — 같은 네트워크의 실물 로봇이 움직일 수 있다. `ROS_DOMAIN_ID` 를 따로 · `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- 이 폴더의 `AGENTS.md` · `CLAUDE.md` 를 팀 저장소 (Lv2_Cartlidar_Assignment) 에 올리기 — 팀 `.gitignore` 가 AI 메모를 뺀다 (팀 결정 2026-10-26).
- 팀 저장소 git 작업 (commit · push · branch) 을 AI 가 직접 하기 — 명령만 안내하고 사람이 실행.

## 아직 정해지지 않은 것
LDS-03 앞뒤 x · 고정 테이프 두께, D435 기울기 (URDF 35.2° vs 팀 문서 10°), D435 · 팔 카메라 실물 camera_info, IMU 칩 위치, 바닥 마찰, 팔 게인 튜닝,
회전 보정 배수 (실측), 빠르게 돌 때 바퀴 속도 잘림 처리 (펌웨어에서 비율 유지하기로 함, 미적용).
