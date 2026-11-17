# 팀원별 기여 · 근거

저장소: https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment
기준 시각: 2026-11-17 (KST), `dev` · `Integration` 병합 기록과 GitHub PR · 이슈 기준.
같이 볼 문서: [architecture.md](architecture.md) (전체 구조, 작성 중) · [issues.md](issues.md) (이슈 표)

팀원마다 **맡은 일** 과 근거 네 가지를 적는다. 근거는 할 일 목록이 아니라, 링크와 결과가 있는 것만 적는다.

| 근거 | 무엇을 적나 |
|---|---|
| 구현 | 직접 작성해 병합된 PR: 링크, 바뀐 기능, 효과 |
| 검증 | 직접 돌린 시험: 조건, 기대, 실제, 원자료 (영상 · 로그), 한계와 개선 |
| 리뷰 | 다른 팀원 PR · 작업에 준 의견: 링크, 구체적인 문제 · 의견, 반영 여부 (후속) |
| 설명 | 전체 흐름과, 내가 맡지 않은 코드 · 설정 · 시험 결과를 이해한다는 근거 |

팀 PR 집계 (GitHub, 2026-11-17)

| 팀원 | GitHub | 병합된 PR | 닫힌 PR (병합 안 됨) |
|---|---|---|---|
| 이창엽 (팀장) | developlcy-oss | 45 | 2 |
| 이홍주 | kanichong | 55 | 0 |
| 온창범 | Onbeom | 91 | 1 |
| 한상준 | WindForce08 | 16 | 0 |

---

## 1. 이창엽 (팀장 · developlcy-oss)

### 1.1 맡은 일

| 영역 | 내용 | 주요 파일 |
|---|---|---|
| 미션 | `mission_node` (단계 실행 · 재시도 · PAUSED), 퍽 측량 · 다시 보기, 위치표 (발제 좌표) | `workspace/ros2_ws/src/mission/` |
| 팔 제어 | `manipulator_node` (pose · pick · execute_skill), IK, 켜질 때 자동 정리 (STOW → DRIVE), 스킬 각본 (넣기 · 꺼내기 · 주사위 놓기), 처짐 보정, E-stop · HOLD 처리, 쥠 판정, 최소 저크 이동 | `control/src/manipulator_node.cpp`, `control/include/control/*.hpp`, `control/config/manipulator.yaml` |
| 정책 배포 | 강화학습 집기 정책 k7 · k8 · k9 를 노드에 올림 (관측 33 조립, 정책별 설정) | `robot_policy/config/policies.yaml`, `robot_policy/config/models/` |
| 강화학습 · 평가 | Isaac Lab 학습 · 노드와 같은 조건 평가 (집기 → 넣기, 꺼내기 → 놓기, 주사위 놓기) | IsaacLab 작업본 (저장소 밖), 결과는 PR · 이슈 본문 |
| 펌웨어 | OpenCR 팔 펌웨어 (E-stop 목표 유지, 게인, HOLD 막힌 관절 풀기, 관절 1 π 보정 v6) | `opencr_firmware/`, `opencr_hardware/` |
| 실물 | 젯슨 브링업, 팔 실측 · 시험 도구 | `workspace/tools/arm_bench/real/` |
| 팀장 | `dev` · `Integration` 병합 (다른 팀원 PR 161 건 병합), 이슈 정리 | - |

### 1.2 구현 (작성 · 병합된 PR)

**미션**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#4](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/4) (10-25) | `center` → `mission` 패키지, `mission_node` 새로 씀 (1,110줄). 순서만 정하고 바퀴 · 팔은 Action 으로 맡김. 단계 문법 `navigate · search · approach · pick · pose · skill`, 실패 시 goal 취소 · 단계별 재시도, E-stop · OpenCR ERROR 면 PAUSED | 미션 · 주행 · 팔 노드의 책임 경계가 이때 정해짐. control 컨테이너 launch 도 같이 만듦 |
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) (11-03) | mission 이 인지 출력 (신호등 · 바코드 · 장소 태그 · 주사위 · 퍽 측량) 을 받아 씀, 측량 실패 시 문제 칸만 다시 보기. manipulator 관절 한계 · STOW · 미션 skill, 팔레트 궤적 `pallet_place_traj.json` | 인지 담당 리뷰 2건 반영 뒤 병합 (아래 1.4 받은 리뷰) |
| [#98](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/98) · [#101](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/101) (11-04) | 다시 보기 거리 `survey.revisit_approach_distance` (0.20 m), 시작 검사 (≥ 0.13 m, TFmini 하한). 위치표를 발제 좌표 (map) 로 | 잘못된 거리면 노드가 시작 단계에서 실패 → 실행 중 사고 방지 |
| [#127](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/127) · [#132](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/132) · [#134](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/134) (11-06) | observe · verify 팔 자세를 계산 자세로 (`box_look` · `barcode_look`), 집기 정책 시작 자세 · 12 s 상한 · 실패 시 `target_look` 부터 재시도, 위치를 잃은 동안 Nav2 단계 대기 (90 s 넘으면 PAUSED) | 전: 위치 잃으면 약 6 분 뒤 FAILED (재개 불가) → 후: 기다렸다 이어 감, 넘으면 PAUSED 로 재개 가능 |
| [#202](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/202) (11-11) | FINE 정렬 팔 자세 READY → ALIGN, `skill.policies` 를 `[toppick_tilt]` 로 | READY 가 상단 카메라에 닿던 문제 (#144) 제거. 없는 모델 하나 때문에 정책 전체가 로드 안 되던 문제도 고침 |

**팔 제어 (manipulator_node · control)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#189](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/189) (11-10) | 켜질 때 DRIVE 로 자동 정리 (J3 → J2 → J1 → J4, HOLD 해제 뒤에만), 스트리밍 (처짐 보정 · 정책) 은 HOLD · E-stop 동안 멈춤 | 11-08 실물에서 매번 손으로 팔을 접던 일을 없앰 (테스트 브랜치에서 실물 자동 정리 성공 후 dev 반영). 병합 당시 빌드 · 단위 시험은 아직이었음 |
| [#204](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/204) · [#205](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/205) · [#209](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/209) (11-11) | 주사위 놓기 집게 벌림 축소 · 처짐 보정, 놓기에도 처짐 보정 스트리밍 (`sag_comp.hpp`), 놓기 실측 도구 | 모의: 주사위 놓기 33.6 % → 90.0 %, 꺼내기 → 놓기 2.8 % → 95.8 % (아래 1.3) |
| [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) (11-11) | 집기 정책 k7 (관측 33) 노드 조립, 정책별 관측 기준 자세 · 시작 자세 · 손가락 값, `skill.toppick_tilt_policy` | 관측 36 만 받던 노드가 다른 관측 형식 정책도 받게 됨. 관측 시험 3개 통과 |
| [#231](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/231) (11-12) | k8 정책, 실측 관절 영점 보정 `ik.joint_offsets`, 정책별 관절 속도 상한 (4.0 rad/s), 놓기 옆 보정, ALIGN = E2 | 6 점 실측으로 남은 오차 평균 6.7 mm. 속도 상한 2.0 → 4.0 으로 k7 노드 조건 92.9 % → 96.6 % (원기둥, 모의) |
| [#236](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/236) (11-12) | 집게 쥠 판정을 위치 범위로, 연속 쥠 신호 `/manipulator/grasped` (30 Hz, 8 / 3 스텝 히스테리시스), 정책 집기 성공에도 쥠 판정 요구 | 실물에서 빈손을 "성공" 으로 보고하던 문제 (11-12 시험) 를 막음 |
| [#244](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/244) (11-12) | E-stop · REARM 때 팔 튐: 붙잡은 목표 유지, REARM 뒤 0.3 rad/s 로 천천히 따라감, 비상정지 중 동작 취소 | 젯슨에서 빌드 · 확인, 튐 줄어듦 (정량값 없음) |
| [#271](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/271) (11-13) | 팔 관절 이동을 최소 저크 곡선으로, 켜질 때 정리 0.1 → 0.3 rad/s | 정리 시간 약 55 s → 단축 (0.2 rad/s 실측 33.1 s) |
| [#273](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/273) (11-14) | 팔레트 궤적 출발 자세 검사, 시작 정리 끝나기 전 명령 거절 | 실물에서 팔꿈치가 퍽 스토리지를 내려친 사고 뒤 넣음 |
| [#282](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/282) (11-14) | 집은 퍽을 0.085 m 들고, 팔레트는 높이 이동 뒤 칸 위에서 수직 넣기 | 엄격 판정 모의: 노드 집기 → 넣기 59.0 % → 83.0 %, 꽉 찬 스토리지 8.0 % → 90.2 % |
| [#286](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/286) · [#287](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/287) (11-14) | STOW J1 1.554 → 2.200 (팀 URDF 충돌 계산, `fold_clearance.py`), `target_look` 퍽 높이를 상자 윗면 기준으로 | 예전 STOW 는 1 열 끝 칸 벽과 −3.5 mm 겹침 → 새 값 최소 9.2 mm. 팔 카메라 높이 오차 8 ~ 22 mm 를 집기 높이에서 뺌 |
| [#290](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/290) · [#291](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/291) · [#293](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/293) (11-14 ~ 15) | 팔레트 v5 실물 10 칸 좌표 반영, 팔 속도 기준 하나 (`arm.max_joint_velocity` 0.8), 칸 위 곧게 내려가기 · 쥠 판정 상한 0.863 | 실물 주사위 배치 실패 (원기둥을 쥐고도 'empty grasp', 칸 축 벗어난 하강) 원인을 고침 |
| [#297](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/297) · [#300](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/300) (11-15) | 상자 5 cm 정지 기준 주사위 놓기 칸별 보정, 고정 동작 집기 접근 높이 0.12 (풀리지 않으면 0.08 · 0.05) | 모의: 5 cm 놓기 87.5 % → 97.5 % (512 대), 세운 직육면체 다시 집기 68.0 % → 84.0 % |
| [#301](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/301) · [#311](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/311) · [#314](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/314) (11-15 ~ 16) | 관절 1 브라켓 재조립에 따라 STOW 값 변경 | 관절 1 모터가 모델과 180° 돌아 달린 것을 실물로 확정 (아래 #316 으로 정리). 한계 1.6 참고 |
| [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) (11-16) | TF 리스너를 작업 중에만 켬 (`tf_idle_off` 30 s) | PC 측정 3.22 → 2.62 % (−19 %). 리뷰에서 "변화가 작다" 는 반대 의견이 있었음 (1.4 받은 리뷰) |

**정책 · 펌웨어 · 도구 · 브링업**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#20](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/20) (10-30) | URDF 261028 · Isaac USD · 경기장 장면 · 매니퓰레이터 강화학습 코드 · 학습 기록 | 팀 Isaac 시뮬 기준 모델. 당시 집기 61 %, 이어 붙이기 0/64 를 그대로 기록 |
| [#245](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/245) (11-12) | 집기 정책 k9 (실측 지연 0 ~ 66 ms 학습), `skill.horizon` 1.0 시험 권장 | 지연을 건 평가에서 k8 원기둥 69.5 % → k9 75.7 %, 지연 0 ~ 33 ms 가정 91.4 / 93.1 % (모의) |
| [#10](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/10) (10-27) | 주행 펌웨어 바퀴 ID · 기어 · 속도 한계 (0.261 m/s), 바퀴 시험 도구 `opencr_wheel_tool.py` | ROS 없이 바퀴 단독 시험 가능 |
| [#233](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/233) · [#242](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/242) · [#243](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/243) (11-12) | `arm_controller` `interpolate_from_desired_state`, 펌웨어 E-stop 목표 유지 · 관절 2 · 3 I 게인 100 · 손목 가속 제한, 대기 자세 검사 그리퍼 범위 | 관절 2 · 3 덜컹 평균 1.3° → 0.07° (실물, 1.3 검증) |
| [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284) (11-14) | HOLD 중 막힌 관절 풀기, 모터별 HW 오류 · 온도 진단 (`/diagnostics`), 토크 ON 전 목표 쓰기 | 온창범 요청 #259 · #277 처리. SIL 3개 추가, colcon test 145 개 통과 |
| [#316](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/316) (11-16) | 팔 펌웨어 v6: 관절 1 만 확장 위치 모드로 π 보정, 역할별 버전 검사, STOW 모델 값 복귀, `j1_flip_bench.sh` | IK · 정책 · 이름 자세를 모델 각도 그대로 쓰게 함. 빌드 · 시험 통과, **업로드 · 실물 확인은 하지 않음** |
| [#246](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/246) · [#255](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/255) · [#298](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/298) | 실물 팔 실측 · 시험 도구 (`kbd_joy.py`, `meas_session.py`, `pick_test.py`), 팔 스킬 teleop `arm_cmd.sh` · `dice.sh` + Isaac 시뮬 스킬 서버, 영점 + 링크 길이 맞추기 `arm_calib_fit.py --links` | 시뮬과 실물을 같은 bash 명령으로 시험. 시뮬에서 집기 → 넣기 → 꺼내기 → 놓기 전 과정, 주사위 눈 6/6 성공 |
| [#238](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/238) · [#299](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/299) | 브링업 `sim` 인자 선언 순서 버그, `split_control:=true` (control 컨테이너를 프로세스 넷으로 나눠 노드별 CPU 측정, #265) | `sim:=` 없이 띄우면 실패하던 문제 해결. 젯슨 측정에서 manipulator_node 22.2 (코어 하나 = 100) 를 찾아 #312 로 이어짐 |

그 밖의 병합 PR: [#281](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/281), [#294](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/294). 닫힌 PR 2 건: [#254](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/254) (#255 로 합침), [#274](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/274) (커밋은 다른 경로로 dev 에 들어감).

### 1.3 검증 (직접 돌린 시험)

원자료 위치: 이슈 · PR 댓글 (링크), 저장소 [`project2/results/real_20261116/`](../results/real_20261116/). 시험 영상 · 학습 기록 일부는 **저장소 밖 IsaacLab 작업본** (`policy/manipulator/기록/`) 에만 있다 (한계 1.6).

**실물**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 켜질 때 자동 정리 (11-08) | 젯슨 cart11, 테스트 브랜치 `feat/test-lcy001`, 조이스틱 없이 `/joy` 리셋으로 HOLD 해제 | HOLD 해제 뒤 STOW 도착 | `stage 1/2 (12.3 s)` · `stage 2/2 (8.8 s)` → `arm at 'STOW'`. 손목은 혼 어긋남으로 묶음에서 뺌 | [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) | 테스트 브랜치에서만 확인 → dev 반영은 #189 (목표 DRIVE) |
| 팔 처짐 · 집게 폭 · 칸 좌표 실측 (11-11) | 바퀴 보드 FAULT 로 브링업 불가 → 팔 OpenCR 직접 연결, 줄자 ±10 mm, 캘리퍼스 | 설계 FK 와 비슷 | 0 자세 설계 311 → 실측 220 mm (−91), 집게 빈손 닫힘 23.8 mm (0.922 rad), 원기둥 쥠 0.571, 칸 3 곳이 한 덩어리로 x +28 · 높이 +52 mm 어긋남 | [#190 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190#issuecomment-6798080681) | 처짐은 한 오프셋으로 안 맞음 (잔차 ±15 ~ 30 mm) → #231 영점 보정, #236 쥠 판정 범위 근거 |
| 강화학습 집기 실물 (11-12 오전) | `skill_enabled:=true`, 팔 Arducam 만, `pick_test.py` (영상 · 관절 CSV · 로그) | 퍽을 쥐고 들기 | **실패 2회**: 상자에 박고 빈손인데 `success: true`, 두 번째는 목표 z 0.171 m 보다 약 5 cm 아래로 파고들어 12 s 시간 끝. 인식 퍽 높이 +25 ~ +64 mm 치우침 | [#190 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190#issuecomment-6811725884), 영상 `pick_1112_*/arm.mp4` (IsaacLab 작업본) | 모의 정책이 실물 처짐 · 지연 · 인식 치우침을 본 적 없음 → 빈손 성공은 #236, 높이는 #287, 지연은 k9 (#245) 로 대응. **k9 실물 재시험은 못 함** |
| 팔 덜컹 측정 (11-12 오후) | 앞 45° 자세, 관절 1 ~ 4 를 ±5° · 0.4 s, 각 · 전류 100 Hz (`meas_session.py` 메뉴 13) | 진동 · 덜컹 원인 찾기 | 모터 진동 없음 (정지 0.088°). 덜컹 원인 = 처진 실측 위치에서 새 동작 시작. 직접 모드 평균 1.3° / 최대 5.1° → 고친 뒤 **브링업 실물 평균 0.07° / 최대 0.08°** | [#165 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/165#issuecomment-6814553855), [#233 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/233#issuecomment-6820146134) | 처음 낸 "D 게인" 제안은 측정 뒤 스스로 철회. 손목 · 집게 관성 출렁임은 엔코더로 안 보여 남음 |
| **실물 미션 (11-16 12:36)** | 젯슨 브링업, `mission_source:=plan` | 첫 단계 observe (`barcode_look`) 팔 자세 | **FAILED**: `observe failed after 2 retries: manipulator: /manipulator/execute_skill goal rejected`. 팔 로그: `startup: arm is not at STOW (0.23 rad) nor 'DRIVE' ... -> not moving` → `execute_skill: startup to 'DRIVE' not finished -> reject` | [`mission_log/261116_1236/mission_log.jsonl`](../results/real_20261116/mission_log/261116_1236/mission_log.jsonl), [`launch_logs/demo_261116_123545.log`](../results/real_20261116/launch_logs/demo_261116_123545.log) | 내가 넣은 안전 검사 (#273 · 알려진 자세 근처에서만 정리) 는 설계대로 동작했지만, 그날 STOW 정의가 여러 번 바뀌어 (#311 → #314 → #316) 실제 팔 자리와 0.23 rad 어긋남 → 팔이 미션에 한 번도 들어가지 못함. 개선: 펌웨어 v6 업로드 · STOW 한 값으로 고정 · 시작 전 준비 확인에 "startup 끝남" 포함 |

**모의 (Isaac Lab, 노드와 같은 조건)**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 주사위 눈 놓기 (11-11) | 2,048 대, L1 ~ L3 × 눈 1 ~ 6, 상자 위치 ±5 mm, 노드 `placeAt` 재현 | 90 % 이상 | 노드 그대로 33.6 % → 벌림 37.6 mm 48.3 % → + 처짐 보정 **90.0 %** | [#204](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/204) 본문 | 원인 (옆 퍽 밀기 · 팔 처짐) 을 나눠 고침. 5 cm 정지 기준으로 다시 맞춤 → #297 (2,048 대 95.8 %, 엄격 판정) |
| 꺼내기 → 놓기 (11-11 ~ 12) | 노드 꺼내기 궤적 그대로, 팔레트 나머지 칸 무작위로 채움 | 98 % | 노드 2 mm 틈 2.8 % → 놓기 처짐 보정 95.8 % → 칸 차례 · 벌림 34.6 mm 까지 **98.2 % (2011/2048)**, v5 (10 칸) **98.3 %** | [#209](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/209), [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) | 칸 차례 조건은 계획기 담당에게 넘겨 반영됨 (#260). 노드 실제 벌림 36.4 mm 로 다시 잰 값은 시간 제한으로 끊김 |
| 집기 정책 k7 → k8 → k9 (11-12) | 노드 조건 (퍽 좌표 한 번 잰 값, 관절 변화 상한, 실측 지연 0 ~ 66 ms), 2,048 ~ 8,192 대 | 집기 → 넣기 95 % 이상 | 지연 없이 k8 원기둥 98.4 %. **지연을 제대로 걸면 k8 69.5 / 82.5 %**, k9 75.7 / 86.3 %, 지연 0 ~ 33 ms 가정 91.4 / 93.1 % | [#245](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/245), [#226 정정 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6817601451) | 처음 낸 면 어긋남별 결과에 지연이 빠져 있었음 → 같은 날 정정 · 재측정 댓글. 98 % 는 못 넘김 |
| 엄격 판정 집기 → 넣기 (11-14) | 옆 퍽 · 운반 중 접촉도 실패로 셈, 400 ~ 512 대 | 예전 판정과 비슷 | 예전 판정이 접촉을 세지 않았음을 찾음. 꽉 찬 스토리지 8.0 % (엄격 1.6 %) → 새 경로 90.2 %, 노드 동작 59.0 % → 83.0 % | [#282](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/282) | 남은 실패 대부분은 집기 단계 |
| 세운 직육면체 다시 집기 (11-15) | `chain_pallet_eval.py --spot`, 세워 두는 자리 D | 상자 접촉 줄이기 | 68.0 → 84.0 % (시드 3: 67.8 → 81.0 %), 상자 접촉 5.5 → 3.8 % | [#300](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/300), [#221 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/221) | 실물 미확인 |

**코드 시험**

| 시험 | 결과 | 근거 |
|---|---|---|
| 펌웨어 SIL · 보드 빌드 | #242 SIL 36 개 통과 · 업로드, #284 SIL 3 개 추가 · colcon test 145 개, #316 opencr_hardware 149 개 (10 skipped) · control 85 개 · 새 SIL 4 개 · `opencr_arm_v6.bin` 빌드 | [#242](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/242), [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284), [#316](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/316) |
| control 단위 시험 | `test_startup_stow`, 관측 33 조립 시험 3 개, control 85 개 (#287 · #297 · #312) | 각 PR 본문 |
| STOW 접기 충돌 계산 | 팀 URDF collision (팔 상자 200 개) 으로 접기 · 펴기 경로 최소 틈: 예전 −3.5 mm → 새 STOW 9.2 mm, 실은 퍽 14.8 mm | [#277 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/277) |

### 1.4 리뷰

**사실 그대로의 집계 (GitHub)**

- 다른 팀원 PR 153 건에 리뷰를 남김 (온창범 85 · 이홍주 52 · 한상준 16). 상태는 APPROVED 159 · DISMISSED 7 (한 PR 에 여러 번 포함).
- **본문이 있는 PR 리뷰는 2 건뿐** ([#1](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/1) "승인", [#2](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/2) "승인합니다") 이고, **줄 단위 리뷰 댓글은 0 건**이다. 팀장으로서 병합 승인은 대부분 본문 없이 했다.
- 다른 팀원 PR 161 건을 직접 병합했다.
- 실제 기술 의견은 PR 이 아니라, 그 PR 들이 가리키는 **이슈 스레드**에 모의 · 계산 근거와 함께 남겼다 (이슈 댓글 60 건). 아래는 그중 의견이 다른 팀원 코드로 이어진 것만 고른 것이다.

**이슈 스레드에서 준 의견 → 다른 팀원 PR 반영**

| 대상 (담당) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| 계획기 칸 배정 (이홍주) — [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) | hybrid 칸 배정이 퍽 모양을 안 봄 → 꺼내기 → 놓기 79 % (모양대로면 98 %). [답 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230#issuecomment-6819961887): 같은 열 차례는 자유, 단 '막는 쌍' (원기둥 칸을 꺼낼 때 닫는 축 쪽 직육면체 칸이 비어 있어야 함) 은 지킬 것, 표로 제시 | **반영**: 칸 모양 배정 → 막는 쌍 · 모양 벌점 [#260](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/260) → v5 로 다시 [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261). 담당자 계산 (Q1 + 무작위 100 문제) 에서 막힌 채 꺼내기 0 |
| 시작 박스 회전 범위 (이홍주 [#235](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/235), 온창범 [#247](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/247)) — [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226) | #235 의 전제 "회전으로 잘라서 남는 각은 집기 정책이 받음" 을 모의로 확인: 면 어긋남 20° 까지는 정렬과 비슷, 30° 넘으면 약 20 %p 떨어짐 → 남는 각이 20° 를 넘지 않게 요청. 첫 값에 지연이 빠진 것을 스스로 찾아 [정정](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6817601451) 후 [재측정](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6823669678) | **반영**: #247 이 "남는 각 20° 넘으면 mission_log 에 기록" 으로 이 기준을 씀. 회전으로 못 맞추는 경우는 [#251](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/251) 로 넘김 |
| `box_look` 7 필드 (온창범) — [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6813575193) | 거리 · 각 지정 필드 추가에 대해: 5 필드 동작 · `min_clearance` · 카메라 장착값이 그대로라 팔 쪽 문제 없음, 영점 보정이 들어가면 같은 거리 · 각에서도 카메라 자리가 조금 바뀔 수 있다고 주의 | **반영**: [#232](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/232) 승인 · 병합 |
| 주행 자세 DRIVE (이홍주 · 온창범) — [#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219#issuecomment-6812219900) | 실은 퍽과 관절 3 이 겹치는 문제에 대해, 계획기 우회 (`avoid_slots`) 대신 DRIVE 를 ALIGN 과 같은 E2 로 바꾸자고 제안 (G 줄 퍽 17.8 mm · 차체 23.4 mm). 라이다 · D435 시야와 footprint 확인 요청 | **반영**: DRIVE = E2 [#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270) (이홍주). `avoid_slots` 는 필요 없게 됨. 실물 확인은 #266 으로 넘김 |
| 누운 직육면체 세우기 정책 (온창범) — [#192](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/192), [#193](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/193) | 다시 잡는 피치 45° 고정, 잡는 점 = 퍽 중심 + 15 mm, 배포 규칙 "`execute_skill` 이름 = `policies.yaml` 키, 외부 데이터 없는 한 파일 ONNX" 를 명세로 줌. 실물 시험에서 젯슨에 옛 `r2e` 가 올라간 것을 찾음 | **반영**: [#215](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/215) · [#217](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/217) (r2g2), 11-16 실물 로그에서 `lieupright_r2g2.onnx` 로드 확인 |
| 손목 (J5) 으로 면 맞추기 제안 (온창범) — [#251](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/251#issuecomment-6876839570) | 관측 · 행동 · 노드 IK 가 모두 바뀌어 재학습이 필요하고, 실물 성공률은 지연 · 처짐에 더 좌우됨 → k9 실물 시험 뒤로 **보류** 판단 | **의도적으로 반영 안 함** (보류로 닫음, 다시 열 조건 명시) |
| 집기 넘어뜨림 · 쥐는 깊이 제안 (온창범) — [#141](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/141) | fit1 대신 k7 (관측 33) 로 정함, 미션 배치 2,048 대 집기 원기둥 98.4 % · 직육면체 97.9 % (지연 없는 조건) | **부분 반영**: 집기 전 퍽 밀림 벌점은 이홍주 [#206](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/206) 에 들어감 |
| 펌웨어 요청 (온창범) — [#259](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/259), [#277](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/277) | 모터별 HW 오류 · 온도 진단, HOLD 가 막힌 목표를 계속 유지하는 문제, STOW 접기 충돌 | **반영 (내가 구현)**: [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284), [#286](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/286) |

**받은 리뷰와 반영 (참고)**

| 내 PR | 리뷰어 · 지적 | 반영 여부 |
|---|---|---|
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) | 온창범: `registered` 모드에서 측량이 실패해도 지난 실행 결과로 계획될 수 있음, `survey.poses` 가 계획기 파일의 복사본이라 어긋날 수 있음 | 반영 (커밋 a573322: 측량 시작 때 지난 등록표 삭제, `approach_poses.yaml` 직접 읽기). 같은 PR 의 이홍주 팔레트 skill 리뷰 6 건 (#75) 은 PR 에 남은 채 병합 — 후속은 #75 담당 이슈로 처리 |
| [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) | 이홍주: 옆 퍽 없음 (0, 0) 을 학습에서 봤는지, 행동 → IK 자세가 학습과 같은지, 관절 속도 상한 2.0 rad/s, 로드 실패가 조용함 | 관절 속도 상한은 [#231](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/231) 에서 정책별 4.0 rad/s 로 반영. 옆 퍽 없음 · IK 자세 일치 근거는 PR 에 답을 남기지 못함 (한계) |
| [#189](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/189) | 한상준 (#144 에서): 목표 자세 STOW → DRIVE, 순서 3 → 2 → 1 → 4 → 5 · 6, 0.1 rad/s | 반영 |
| [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) | 온창범: CPU 0.68 % 변화는 의미 없으니 반려 의견 | 반영하지 못하고 다른 팀원 (한상준) 이 병합. 의견을 놓고 합의한 기록은 없음 |

### 1.5 설명 (전체 흐름 이해)

전체 구조는 [architecture.md](architecture.md) 에 정리한다 (작성 중). 내가 이해하고 쓰는 흐름은 아래와 같다.

```
/mission/start ─▶ mission_node ──(/mission_planner/plan)──▶ mission_planner (칸 배정 · 싣기/내리기 차례)
                     │
                     ├─▶ drive_node : /drive/navigate (Nav2 navigate_to_pose · spin) · approach COARSE/FINE (TFmini 0.13 m)
                     ├─▶ manipulator_node : /manipulator/pose · pick · execute_skill
                     │        ├─ 각본 스킬 (pallet_load / unload, place_<L>, target_look, box_look)
                     │        └─ 정책 스킬 (robot_policy: policies.yaml 의 ONNX, 30 Hz)
                     │              ▼
                     │        ros2_control arm_controller ─▶ opencr_hardware ─▶ OpenCR 팔 펌웨어 (STATE/CMD, HOLD/E-stop)
                     └─◀ perception (/perception/pucks · place_box · 태그) · localization (/localization/status)
```

내가 맡지 않은 코드 · 설정 · 결과를 읽고 판단한 근거:

| 근거 | 내가 맡지 않은 부분 | 무엇을 이해 · 판단했나 |
|---|---|---|
| [#194](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/194) 정차 간격 (drive · planner · policy) | 주행 `drive.yaml`, 계획기 `FINE_GAP`, TopPick 학습 배치 | "코스트맵이 12 cm" 라는 질문을 추적해 원인이 Nav2 `inflation_radius` 가 아니라 TFmini 최소 거리로 정한 `min_target_distance 0.13` · `stop_margin 0.03` · 계획기 `0.1272` 임을 밝힘. 5 ~ 6 cm 로 줄이면 바뀌는 곳 (TFmini · obstacle_stop · 계획기 · 학습) 을 비교해 "정차는 그대로, 팔 정책을 0.1272 로 다시 학습" 으로 결론 (#195) |
| [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) · [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226) 계획기 | `mission_planner` 칸 배정 · 회전 범위 | 계획기 hybrid 운영 (#224) 을 읽고 모양 무시 문제를 찾음, 계획기 담당이 쓸 수 있는 조건 (막는 쌍 표, 남는 각 20°) 으로 바꿔 전달 |
| [#128](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/128) · [#143](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/143) 인지 | perception 팔 카메라 | 칸에 못 앉는 원인이 손 안 퍽 위치 오차임을 모의로 보이고 (오차 0 mm 면 세운 직육면체 30/1) 인지 쪽에 ~1 mm 측정 가능성을 물음. 실물에서 퍽이 안 보이는 원인을 `arm_camera_info_url` 이 비어 K = 0 camera_info 가 나가 perception 이 거부한 것으로 찾음 → 온창범 #146 으로 해결 |
| [#114](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/114) Nav2 | 라이다 장착 · Nav2 설정 | URDF 로 스캔 면 0.082 m vs 상자 윗면 0.070 m (여유 12 mm) 를 계산해 유령 점 위험을 알리고, 팔이 닿는 거리로 정지 거리 · footprint 값을 제안 |
| 11-09 담당 노드 검토 → [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) | 다른 노드와의 연결 (action · 메시지 필드 · yaml 키) | 연결은 문제없고, NaN 이 `withinLimits` 를 통과해 팔 명령까지 가는 점, E-stop 뒤 스트림이 최대 0.45 rad 앞서 남아 REARM 때 튈 수 있는 점을 찾음 → #189 · #244 |
| [#265](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265) CPU (bringup) | 브링업 컨테이너 구성 | 한 프로세스라 노드별 CPU 를 못 가르는 문제를 `split_control` 로 풀고 (#299), 젯슨 측정으로 manipulator `/tf` 구독이 큰 몫임을 찾아 #312 로 줄임 |
| 11-16 실물 로그 해석 | mission · planner · Nav2 브링업 | 같은 로그에서 계획기 서버 없음 · Nav2 lifecycle 대기로 처음 시작이 거절된 것과, 팔 시작 정리 거절로 observe 가 실패한 것을 구분 (1.3 표) |

### 1.6 한계 (솔직하게)

1. **11-16 실물 미션에서 팔이 한 번도 동작하지 못했다.** 첫 단계 observe 의 `execute_skill` 이 `goal rejected` 로 3 번 거절돼 미션이 FAILED. 원인은 팔이 STOW 에서 0.23 rad 어긋나 켜질 때 정리가 시작되지 않은 것이고, 그 배경은 같은 날 STOW 정의를 여러 번 바꾼 것 (#311 → #314 → #316) 이다.
2. **실물 집기 → 넣기 · 주사위 놓기를 처음부터 끝까지 성공시킨 기록이 없다.** 실물 강화학습 집기는 11-12 에 2 번 실패했고, 그 뒤 k9 · `skill.horizon 1.0` 실물 시험은 하지 못했다. 실물 주사위 배치는 11-13 에 pick · place 한 번 성공이 전부다.
3. 성공률 수치 (90 ~ 98 %) 는 대부분 **Isaac 모의** 다. 실측 지연을 빼고 잰 평가를 한 번 올렸다가 정정했다 (#226). 지연을 넣으면 집기는 아직 85 % 안팎이다.
4. **관절 1 모터 180° 문제** 를 펌웨어 v6 (#316) 로 정리했지만 업로드 · 실물 확인을 하지 못했다. 영점 · 링크 길이 재보정 (#292 · #295) 도구는 커밋하지 않고 stash 에 있다.
5. 계획기 정지 거리 (12.7 cm) 와 노드 놓기 보정 기준 (5 cm, #297) 이 아직 맞지 않는다.
6. PR 리뷰는 대부분 본문 없는 승인이었다. 의견은 이슈에 남겼지만 PR 화면에서 리뷰 근거가 보이지 않는다. 받은 리뷰 중 #214 의 일부 질문과 #312 의 반려 의견에는 답을 남기지 못했다.
7. 시험 영상 · 학습 기록 일부가 저장소 밖 (IsaacLab 작업본, 임시 릴리스 `rl-20261110`) 에 있어 다른 사람이 바로 열어 볼 수 없다.

---

## 2. 이홍주 (kanichong)

> 아래는 틀만 있다. 내용은 본인이 작성한다.

### 2.1 맡은 일
(본인 작성 필요)

### 2.2 구현
(본인 작성 필요)

참고 — 병합된 PR 번호 (GitHub 기준, 55 건): #1, #11, #13, #17, #50, #60, #61, #62, #67, #70, #79, #99, #104, #105, #106, #117, #136, #138, #142, #149, #152, #156, #167, #169, #170, #196, #200, #206, #208, #211, #213, #222, #224, #227, #235, #237, #239, #260, #261, #263, #264, #267, #268, #269, #270, #272, #283, #285, #288, #289, #307, #308, #313, #315, #320

### 2.3 검증
(본인 작성 필요)

### 2.4 리뷰
(본인 작성 필요)

### 2.5 설명
(본인 작성 필요)

### 2.6 한계
(본인 작성 필요)

---

## 3. 온창범 (Onbeom)

> 아래는 틀만 있다. 내용은 본인이 작성한다.

### 3.1 맡은 일
(본인 작성 필요)

### 3.2 구현
(본인 작성 필요)

참고 — 병합된 PR 번호 (GitHub `Onbeom` 기준, 91 건): #2, #3, #5, #7, #8, #9, #16, #18, #19, #21, #22, #23, #24, #25, #26, #27, #28, #29, #32, #33, #42, #44, #45, #46, #47, #48, #51, #54, #56, #57, #58, #68, #72, #80, #90, #91, #95, #96, #97, #100, #102, #103, #120, #122, #123, #124, #125, #126, #129, #130, #131, #137, #140, #146, #147, #148, #150, #153, #158, #159, #162, #164, #171, #174, #175, #176, #180, #181, #182, #186, #198, #203, #210, #212, #215, #217, #223, #228, #232, #234, #240, #247, #250, #252, #253, #256, #280, #296, #304, #309, #310

### 3.3 검증
(본인 작성 필요)

### 3.4 리뷰
(본인 작성 필요)

### 3.5 설명
(본인 작성 필요)

### 3.6 한계
(본인 작성 필요)

---

## 4. 한상준 (WindForce08)

> 아래는 틀만 있다. 내용은 본인이 작성한다.

### 4.1 맡은 일
(본인 작성 필요)

### 4.2 구현
(본인 작성 필요)

참고 — 병합된 PR 번호 (GitHub 기준, 16 건): #6, #12, #76, #115, #155, #163, #166, #179, #207, #241, #257, #258, #302, #306, #318, #319

### 4.3 검증
(본인 작성 필요)

### 4.4 리뷰
(본인 작성 필요)

### 4.5 설명
(본인 작성 필요)

### 4.6 한계
(본인 작성 필요)
