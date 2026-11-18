# 261112 비스듬히 서기 (mission) mock 시험

팀 저장소 작업 폴더 `~/git/Lv2_Cartlidar_Assignment_oblique` (브랜치 `fix/mission-oblique-stand`, origin/dev c689a38 에서). 커밋 안 함.
빌드는 작업 폴더 `workspace/ros2_ws` 에서, 비교 기준 (origin/dev 그대로의 mission) 은 이 폴더 `base_src` · `base_build` · `base_install`.
이 폴더의 스크립트 (`run_mock.sh` · `run_all.sh` · `check_oblique.py` · `mock_robot_oblique.py`) 는 확인용이라 팀 저장소에 넣지 않는다.
`mock_robot_oblique.py` 는 팀 `mission_planner/test/mock_robot.py` 사본을 늘린 것 (원본은 그대로).

## 무엇을 바꿨나 (mission 패키지)
- `oblique.enabled` (기본 false) 를 켜면 장소 상자 재기 (`measure_box <L>`) 가 그 장소 `approach_box` 앞 (대기 자리) 으로 옮겨 붙고
  앞 D435 재기 → 제자리 회전 → 축을 따라 보기 자리까지 align BOX → 팔 box_look (0.44 m / 50°) → 기존 팔 재기 순서로 바뀜.
- verify 앞에 끼우던 `approach_box 0.13` 대신 `approach_view <L>` (보기 자리 TFmini 목표), verify box_look 은 7 칸 (0.40 m / 55°) 로 잰 중심을 봄.
- 계산은 `include/mission/oblique.hpp` (헤더만) + 단위 시험 `test/test_oblique.cpp`.

## mock 이 흉내 내는 것 (mock_robot_oblique.py)
차체 map 자세를 들고 odom → base_link TF 를 냄. navigate 목표가 0.25 m 안이면 icr_x (−0.0658) 둘레 제자리 회전, 멀면 순간 이동.
align box_only 는 렌즈 (base x 0.13) 에서 x 축 빔을 진짜 상자 (지도 칸 + box_shift, box_rot) 에 쏴서 렌즈 → 면 = 목표가 되게 옮김.
measure_front 는 진짜 상자를 odom 으로 냄 (status camera front). 팔 measure 는 원본과 같음.

## 시험 결과

모두 **모의 시험** (가짜 주행 · 팔 · 인식, 계획기 · mission_node 는 진짜). 실물 · Isaac 아님.
상자 자세는 세 장소 모두 같은 box_rot · box_shift (지도 축). 계획 = `problem_q1_example.yaml` (장소 L2 · L1 · L3 순서).

| # | 확인 | 예상 (값의 뜻 · 예상과 이유) | 결과 |
|---|---|---|---|
| 1 | 빌드 (`colcon build --packages-up-to mission mission_planner`) | 종료 코드 0, 새 코드 경고 0. 헤더 · 멤버만 늘려 다른 패키지 영향 없음 | 종료 코드 0, 경고 0 (`build.txt`, mission 37 s) |
| 2 | 단위 시험 (`colcon test --packages-select mission`) | 통과 수 / 전체: 기존 4 개 실행 파일 그대로 통과 + `test_oblique` 4 개 (접기 · 회전 · 보기 거리 · 빔 계산과 TFmini 목표 일치) 통과, 실패 0 | 5 개 실행 파일 27 경우 모두 통과, 실패 0 (`test_oblique` 4/4, `test.txt`) |
| 3 | 기준 run (origin/dev mission, `run_base`) | DONE. 끈 상태 비교 기준 | DONE 55.2 s (`run_base/`) |
| 4 | 끔 (`run_off`, oblique false) vs 기준 | 단계 · navigate · align · 팔 skill · 모드 · 재기 기록이 기준과 모두 같음 (11 항목 같음). measure_front · oblique 줄 0, box_look 5 칸만. 기본값 false 면 새 가지를 안 타서 | 11 항목 모두 같음 (단계 115 · navigate 18 · align 29 · skill 64 · set_mode 100 …), measure_front · oblique 0, box_look 5 칸만. 14/14 (`run_off/check.json`) |
| 5 | 켬 rot 0 shift 0,0 (`run_r0`) | 장소마다 measure_front 1 번 (3). 상자 중심이 대기 자리 축 위라 회전 0 (spin 0), beta 0, R_look 0.5853 m, TFmini 목표 0.3528 m, tfmini. 접근 뒤 중심 base x = R_look (±1 mm). 놓기 box_look :0.440:50.0, verify :0.400:55.0. DONE | measure_front 3 (L2 · L1 · L3), spin 0, beta 0, R_look 0.5853, 목표 0.3528, tfmini. 접근 뒤 중심 base (0.5853, 0.0000), 이동 +0.100 m. box_look 7 칸 확인, DONE 56.2 s. 20/20 |
| 6 | 켬 rot 20 shift 0.03,−0.03 (`run_r20`) | 중심이 축에서 3 cm 옆이라 회전 약 ±2.2~2.4°, 보기 beta 17.8~22.4° (돈 만큼 바뀜) → 모두 tfmini (≤ 30°), 목표 약 0.37 m. 공식 차 ≤ 1 mm · 0.05°. DONE | 회전 L2 +2.20° · L1 −2.20° · L3 −2.38°, 보기 beta 17.80 · 22.20 · 22.38°, 모두 tfmini, 목표 0.3741 · 0.3757 · 0.3758. 공식 차 최대 4.3e-5, 접근 뒤 중심 옆 0.0000 m. verify 접근 0.3751 (beta 20°). DONE 56.8 s. 20/20 |
| 7 | 켬 rot 40 shift −0.05,0.05 (`run_r40`) | 회전 약 ±3.6~4.1°, 보기 beta 약 36~44° → 모두 tfmini_wide + 경고 줄 (장소 수 + verify 접근 수). 그래도 같은 TFmini 목표로 진행, DONE | 회전 −4.08 · +4.08 · +3.57°, 보기 beta 44.08 · 35.92 · 36.43° → 모두 tfmini_wide, 경고 5 줄 (놓기 3 + verify 접근 2). 목표 0.3551 · 0.3694 · 0.3687, DONE 56.4 s. 20/20 |
| 8 | 켬 rot −45 shift 0,0 (`run_rm45`) | 접으면 beta ±45° (경계), 회전 0, R_look 0.6278 m, 목표 0.3528 m, tfmini_wide. DONE | beta L2 −45° · L1 +45° · L3 +45° (접기 경계), 회전 0, R_look 0.6278, 목표 0.3528, tfmini_wide, 경고 5 줄, DONE 56.6 s. 20/20 |

시나리오 실행: `run_all.sh` (요약 `summary.txt`, 장면마다 `run_<이름>/` 에 mission.log · mock_out.json · check.json · log/<에피소드>/mission_log.jsonl).
- 도메인은 87 (지시는 77): 같은 PC 의 다른 세션이 77 에서 perception 재생 (pbfront) 을 돌려 첫 시도에서 /perception/place_box · TF · measure_front 호출이 섞였음.
- 첫 시도에서 mock 이 순간 이동 직후 TF 를 늦게 내 mission 이 옮기기 전 자세로 계산함 → mock 이 옮긴 뒤 TF 를 바로 내고 0.2 s 기다리게 고침 (mock 문제, mission 은 그대로).
- r40 확인은 처음 인자 (`--shift -0.05,0.05`) 오류로 안 돌아 같은 결과 폴더로 다시 확인함 (run_all.sh 고침).

## 한계 (이번에 안 한 것)
- 놓기는 지금 방식 그대로: 보기 자리에서 잰 뒤 계획기 `approach_box 0.13` 이 지금 (돈) 방향으로 TFmini 0.13 까지 다가가 놓음.
  상자 요에 따라 앞 끝 → 상자 36~117 mm 로 서는 놓기 자리는 odom 직진 (drive #225) 이 있어야 해서 안 함.
- |beta| > tf_max_beta_deg 에서도 TFmini (align BOX) 를 그대로 씀 (odom 직진 없음). 빔이 모서리 근처에 닿으면 실제 거리가 어긋날 수 있음 (mock 빔 계산은 정확한 정사각형이라 이 위험을 못 봄).
- 퍽 측량 (survey_view, 시작 박스) 에는 넣지 않음: 시작 박스는 퍽이 놓여 있어 앞 D435 윗면 재기 (장소 상자용) 를 그대로 쓸 수 있는지 perception 확인이 먼저이고,
  survey 는 박스마다 goto + approach 단계가 미리 펼쳐져 있어 측정 · 회전 · 접근을 넣으면 단계 구조가 크게 바뀜.
- 장소 첫 놓기 앞에 approach_box 가 없는 계획 (지금 계획기는 항상 넣음) 이면 재기가 예전 자리 (꺼내기 앞, 정밀 자리) 에서 같은 흐름으로 돎 (회전 · 후진 포함).
- verify 접근 (approach_view) 은 돌지 않고 대기 자리 방향 그대로 다가감 (beta = 잰 상자 요 − 지금 차체 요).

- 11-12 검토(261111 1) 지적: 기록 이름 `odom_fallback` 이 실제 동작(같은 TFmini)과 달라 `tfmini_wide` 로 바꿈. 위 표의 이름도 바꿔 적음. 바꾼 뒤 단위 32/32, run_r40 · run_rm45 다시 20/20 · 20/20(`summary_rerun_tfmini_wide.txt`).
