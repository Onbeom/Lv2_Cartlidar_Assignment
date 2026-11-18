# 매니퓰레이터 강화학습 이어 하기 묶음 (2026-11-06)

다른 PC 에서 지금과 같은 진행도 (정책 · 체크포인트 · 보상 설정 · 평가 도구 · 기록) 로 이어 학습 · 평가하기 위한 묶음.

## 1. 풀기 · 놓을 곳
압축 안 `lv2_urdfusd/` 를 Isaac Lab 3.0 의 `scripts/` 아래에 그대로 둔다 (상대 경로가 맞아야 함).
```
<IsaacLab>/scripts/lv2_urdfusd/
  isaac/                     팀 로봇 설정 (읽기만): waffle_4wd_isaac/ robot_cfg.py, robot_params.json, urdf/, usd/
  policy/manipulator/        매니퓰레이터 과제 · 학습 · 평가 (여기서 실행)
```
- `policy/manipulator/waffle_isaac/robot_cfg.py` 가 `../../isaac` 의 팀 설정 · USD · URDF 를 불러옴. 두 폴더의 상대 위치를 바꾸지 말 것.
- 원래 PC: Isaac Lab `release/3.0.0` 저장소, Docker 컨테이너 `isaac-lab-base` 안 `/workspace/isaaclab` (= 호스트 `~/dockerWorkspace/IsaacLab`). GPU 8 GB (RTX 노트북).
- `run_rl.sh` 는 `ISAACLAB_DIR` (기본 `/workspace/isaaclab`, 없으면 `~/dockerWorkspace/IsaacLab`) 또는 `ISAACLAB_PY` 로 Isaac Lab 파이썬을 찾음.
- `sync_to_docker.sh` 는 원래 PC 전용 (실행하지 말 것).

## 2. 지금 쓰는 정책 (`policy/manipulator/policies/`)
| 용도 | 파일 | 이어 학습 체크포인트 (`logs/rsl_rl/...`) | 결과 (48 대, 상자당 2 개, 손 안 퍽 가정) |
|---|---|---|---|
| **세운 원기둥 45° 집기 (기본)** | `sidepick45_fit1.pt` = `sidepick45_latest.pt` | `waffle_sidepick45/2026-11-06_13-10-50/model_6348.pt` | 가운데 배치 2 개 다 25 ~ 28/48, 퍽 자리 무작위 22/48 |
| **세운 직육면체 45° 집기 (기본)** | `sidepick45box_fit1.pt` = `sidepick45box_latest.pt` | `waffle_sidepick45box/2026-11-06_13-11-51/model_28790.pt` | 가운데 19 ~ 25/48, 무작위 12/48 |
| 원기둥 push1 (시험, 안 씀) | `sidepick45_push1.pt` | `waffle_sidepick45/` 가장 최근 폴더 | 무작위 17/48 |
| 직육면체 push1 (시험, 안 씀) | `sidepick45box_push1.pt` | `waffle_sidepick45box/` 가장 최근 폴더 | 무작위 15/48 |
| 원기둥 d1 (fit1 이전) | `sidepick45_d1.pt` | `waffle_sidepick45/2026-11-05_21-19-00/model_5849.pt` | 가운데 26/48 |
| 직육면체 d9 (fit1 이전) | `sidepick45box_d9.pt` | `waffle_sidepick45box/2026-11-06_01-16-21/model_28291.pt` | 가운데 11/48 |
| 그 밖 | `sidepick45_SP45c_4940.pt`, `sidepick45box_r35.pt`, `lieroll45_*` 등 | `waffle_sidepick45/2026-11-04_16-00-30/model_4940.pt` 등 | 기록 참고 |
- `.pt` = TorchScript (연결 평가 · 실물용), `.onnx` 일부. 체크포인트 `model_*.pt` 는 옵티마이저 포함 (이어 학습용), 같은 폴더 `params/` 에 학습 때 환경 · PPO 설정 사본.

## 3. 과제 · 관측 · 행동 (`waffle_isaac/sidepick_env.py`, `sidepick_env_cfg.py`)
- 과제: `Waffle-SidePick45-Direct` (원기둥 50 % + 세운 직육면체 50 %), `Waffle-SidePick45Box-Direct` (직육면체만). 30 Hz (물리 120 Hz, decimation 4), 에피소드 6 s.
- 행동 4: 잡는 점 직교 이동 (스텝당 최대 5 mm = 15 cm/s, 감쇠 최소제곱 IK, 집게 45° 아래 · 바깥 자세 유지) + 집게 열고 닫기.
- 관측: 팔 관절 위치 · 속도, 손가락, 인식 퍽 좌표 (에피소드마다 ±4 mm 치우침 + 스텝 ±1 mm), 잡는 점, 차이, 집게 축 2 개, 옆 퍽 상대 위치 (두 번째 퍽 = 0), 직육면체 여부, 이전 행동.
- 배치 (`dock_layout`): 상자 앞면 = 로봇 앞면 (TFmini, base_link x 0.140) + 0.15 ~ 0.19 m, 좌우 ±4 cm, 요 ±5°. 50 % 두 번째 퍽 (옆 퍽 없음). 집을 퍽 관절1 축 0.28 ~ 0.365 m (밖이면 상자째 옮김).
  - `dock_rand_prob` 0.7 (push1 부터): 두 퍽 자리 · 간격 (45 mm 이상) · 요 무작위 (닿는 곳만, 상자 위의 약 52 %). **fit1 은 이것 없이 (0.0) 학습** — 가운데 · 8 cm, 80 % 옆으로 나란히.

## 4. 보상 · 가중치 (45° 집기, 지금 코드 기본값)
| 항목 | 값 | 내용 |
|---|---|---|
| rew_reach / reach_fine | 1.0 / 1.0 | 잡는 점-목표 거리 tanh (10 cm / 2 cm) |
| rew_horizontal · camera_up · point · align | 0.5 · 0.3 · 0.5 · 0.5 | 집게 45° 자세 · 카메라 위 · 퍽 향함 · 직육면체 면 맞춤 (12 cm 안) |
| rew_grasp · lifted · lift_height | 3 · 6 · 10 | 쥠 (2 cm 안 + 손가락 범위) · 듦 · 5 cm 까지 높이 |
| rew_puck_tilt · toppled | −2 · −2 | 가까울 때 퍽 기울기 · 넘어짐 |
| rew_topple_end | −30 + 종료 | 목표 퍽 넘어뜨림 |
| rew_nb_touch · nb_touch_end · nb_near · nb_topple · neighbor_moved | −10 · −30 + 종료 · −3 (3 cm 안) · −50 + 종료 · −20 /m | 옆 퍽 |
| rew_arm_box · body_box · base_move | −2 · −20 + 종료 · −10 + 종료 | 상자 접촉 · 차체 접촉 · 차체 밀림 |
| rew_arm_body_end | −50 + 종료 | 팔 · 손가락 ↔ 로봇 몸체 1 N |
| rew_self_end | −100 + 종료 | 자기 충돌: 집게 · 쥔 퍽 ↔ 윗팔 · 아랫팔 선분 55 mm, 관절3 89°, **팔끼리 충돌 상자 5 mm 넘게 겹침** (`arm_selfcol.py`, 팀 USD 는 팔끼리 충돌을 뺐음) |
| rew_j3_bend | −5 / 0.1 rad | 관절3 80° 넘은 만큼 |
| rew_held_tilt | −4 / 5° | 든 퍽 기울기 |
| rew_grasp_dz | −2 / 5 mm | 잡는 높이 |
| **rew_grasp_fit** (fit1 부터) | −3 / 2 mm | 든 동안 실제 퍽 중심 − '잡는 점 15 mm 아래' (손가락 축 뺌) = 놓기 가정 오차 |
| rew_pre_push (push1 부터) | −3 / 5 mm | 들기 전 퍽이 밀린 거리. **fit1 은 0** |
| rew_close_still (push1 부터) | −1 / 2 cm/s | 닫는 동안 잡는 점 속도. **fit1 은 0** |
| rew_action_rate · joint_vel · base_tilt · fall | −0.02 · −1e-3 · −8 · −5 | |
- 집기 중 관절3 목표 상한 1.3 rad (74.5°, `pick_j3_max`, 실물 한계 ±91.7° 와 별개).
- **fit1 을 그대로 재현하려면** `sidepick_env_cfg.py` 의 `WaffleSidePick45EnvCfg.__post_init__` 에서 `dock_rand_prob`, `rew_pre_push`, `rew_close_still` 를 0 으로.

## 5. PPO (`waffle_isaac/agents/rsl_rl_ppo_cfg.py`, `WaffleSidePickPPORunnerCfg`)
배우 · 비평 MLP [256, 128, 64] elu, 관측 정규화, 시작 표준편차 0.5, 학습률 3e-4 (adaptive, KL 0.01), gamma 0.98, lambda 0.95, entropy 0.003, clip 0.2, epoch 5, minibatch 4, 환경당 24 스텝, 저장 50 반복마다. 환경 256 개, 이어 학습 500 반복 (약 12 분 혼자 / 동시 2 개면 각 25 분).

## 6. 실행 (policy/manipulator 에서)
```bash
./run_rl.sh sidepick45 256 500 <체크포인트>        # 원기둥 이어 학습 -> logs/rsl_rl/waffle_sidepick45/<시각>/
./run_rl.sh sidepick45box 256 500 <체크포인트>     # 직육면체
./run_rl.sh sidepick45_diag 64 [정책 .pt]          # 단독 집기 진단 (인자 없으면 가장 최근 학습을 policies/sidepick45_latest.pt 로 내보냄)
./run_rl.sh chain45_eval 48 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --sidepick policies/sidepick45_fit1.pt
./run_rl.sh chain45_play 6 --per_box 2 --fill_order lr --occ 1.0 --est --seconds 40 --render_every 2   # 창 (학습 중 금지)
TAG=fit2 CK_CYL=latest CK_BOX=latest ./run_fit.sh   # 학습 -> 진단 -> 상자당 2 개 평가 한 번에 (fit_summary.log)
./run_push.sh                                       # push1 실험 (기준 + 학습 + 평가)
./bench_speed.sh                                    # 환경 수별 학습 속도
```
- `chain45_play.py` 주요 옵션: `--per_box 2` (상자당 2 개), `--fill_order lr` (3 열 -> 1 열, 칸 0 -> 3), `--occ 1.0` (앞 칸 퍽 다 참), `--est` (손 안 퍽 위치 가정, 실물 조건), `--est_z box`, `--est_meas σ`, `--perc_bias mm`, `--box_prob 1.0`, `--lie_regrasp --lie_yaw radial|side --lie_nb tan` (누운 직육면체 A안), `--render_every N`, `--speed/--row_speed/--empty_speed`.
- 평가 출력: 끝 분류, '다 채운 대' (두 퍽 다 앉음), 손 안 퍽 가정 오차, 팔끼리 충돌 상자 겹침 (5 mm 넘으면 멈춤).

## 7. 지금 상태 · 다음 (자세한 것은 `기록/`, `정책_학습기록.md`, `세션인계.md`)
- 상자당 2 개 성공 약 50 % (가운데 배치). 실패 = 칸에 못 앉음 (손 안 퍽 깊이 2 ~ 5 mm 어긋남, 칸 틈 1 mm) + 집기 중 넘어뜨림 (48 대 중 6 ~ 12).
- 확인한 것: 손 안 퍽 위치를 재면 직육면체 못 앉음 16 -> 1 (`--est_meas 0`), 인식 치우침 ±1 mm 면 원기둥 35/48. push1 (밀기 · 닫는 동안 속도 벌점) 은 효과 없음 (닫는 동안 속도 15 cm/s 그대로).
- 다음 (강화학습): ① 닫는 동안 팔 이동을 행동 규칙으로 막고 이어 학습, ② 누운 직육면체 다시 집기 (새 배치), ③ 주사위 눈 놓기 새 과제, ④ 실물용 (교사-학생).
- 퍽 자리 무작위에서 정지 거리 고정이면 상자 위의 약 52 % 만 45° 로 닿음.
- 규칙: 학습 기준 500 반복, 팔-몸체 · 팔끼리 충돌 금지, git commit · push 는 자동으로 하지 않음.
