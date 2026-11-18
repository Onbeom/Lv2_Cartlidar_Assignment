# PR 없던 사용자 브랜치 3개 처리 · 2026-11-01 (사용자 '3개 모두 처리')

| 브랜치 | 처리 | 확인 방법 |
|---|---|---|
| `fix/perception-selfbox-1711`(3e8d595, push 됨) | 그대로 PR(dev · 열린 PR 과 충돌 0) | URDF 기하(`selfbox/extent.py`) + 빌드 · 단위 시험 |
| `feat/perception-rear-obstacle`(45242af, dev 와 충돌) | 새 브랜치 `feat/perception-rear-obstacle-dev`(dev 위, 작업 트리 `_rear2`, 패치 `rear2.patch`). 충돌 풀이: 시험을 따로 gtest 대상(test_rear_obstacle)으로 · launch 설명 줄 자리 옮김(열린 PR #24 · #25 와 줄이 겹치지 않게) | 10-27 판정 렌더(`261027_2058/results/judge_render`) 재생을 새 빌드로 다시 → 10-27 결과와 같은지 + 단위 시험 |
| `fix/perception-barcode-cfg`(580a25b, push 됨) | 그대로 PR(충돌 0) | C++(이 값) ↔ Python locate_v2(R3 판정 7차 통과 설정) 대조, 판정 7차 렌더 × 4 조건 + 단위 시험 |

합친 트리 `_merge16`(= 열린 PR 13개 + 위 3개, 순서대로 충돌 0, ref refs/keep/all16 8b6af2c), 빌드 `build_merged` 등.

## 확인 결과 (모의 · 도구 확인)
| 항목 | 기준 | 예상 (실행 전) | 결과 |
|---|---|---|---|
| selfbox: 라이다 높이 띠(z 0.048~0.088) 로봇 외곽 | 상자 안 | 판정 모델 261028 · 1711 모두 y ±0.1676 · x 안 → dev ±0.1626 는 5 mm 좁음, 새 ±0.1776 는 1 cm 여유 | (실행 전 계산함) y ±0.1676, x −0.1547~0.1280(261028) / ±0.2185(1711) → 새 상자 안, dev 상자 밖 |
| 단위 시험(합친 트리) | 모두 통과 | 162 + 후방 장애물 7 = 169 | **170/170** (예상보다 1 = 새 gtest 대상의 ctest 줄) |
| 후방 장애물 재생(10-27 판정 렌더 630 프레임) | 10-27 과 같음(검출 350/350 · 헛 0/270 · 오차 중앙 0.6 mm) | 같음 — 계산부 코드는 그대로, 바뀐 것은 빌드 줄 · launch 줄| **같음**: 630 프레임 무응답 0, 검출 350/350, 헛 0/270, 오차 중앙 0.6 mm · p95 16 mm, 사각 장면 2 · 종류별 값 모두 10-27 과 같음 |
| 바코드 C++ ↔ Python(판정 7차 렌더 × clean · 5장 평균 씨앗 0 · 1 · 2) | 장면마다 검출 목록 같음, 판정 값 같음 | 같음(10-26 45차 비교도 4656 장면 같았음) · 판정 7차 값 벽 92/92 · 윗면 45~46/46 · 헛 0| **같음**: 4 조건 × 204 장면 = 816 장면 검출 목록 모두 같음(검출 159 · 157 · 157 · 158), 위치 차 0.0 mm, 점수 차 ≤ 1.9e-4. 판정 값 C++ = Python = 판정 7차: 벽 92/92 × 4, 윗면 46 · 45 · 45 · 46/46, 헛 0, 오차 최대 2.52 · 2.57 · 3.44 · 2.76 mm |

브랜치 단독 빌드 1차: 후방 장애물 10 개 중 dev 기존 시험 SignalLight.U5b_OcclusionRandom1000 이 60 s 제한 넘김(바코드 대조 6 작업과 동시, CPU 부하) — 다시 돌림. selfbox · barcode-cfg 는 그때 perception_survey 패키지가 없어 `--packages-up-to perception_survey` 가 실패(코드 문제 아님) — perception 만으로 다시. `run_build2.sh`.

브랜치 단독 다시(`run_build2.sh`, 2026-11-01): 후방 장애물(dev + 변경) **136/136**(종료 코드 5 = dev perception_survey 시험 0개, #21 이 고침), selfbox-1711 **101/101**, barcode-cfg **87/87**(둘 다 10-26~27 기준이라 시험 수가 적음).
**세 브랜치 모두 확인 통과(모의 · 도구 확인).** PR 본문 `PR_selfbox1711.md` · `PR_barcodecfg.md` · `PR_rear.md`.

## 추가: 측량이 위치 추정 상태 받기 (YOLO 1101 2 재위치 노드 통보, 묻지 않고 진행)
브랜치 `fix/perception-survey-loc-status`(dev 위, 작업 트리 `_locstatus`, 패치 `locstatus.patch`): `perception_survey/localization.py`(새) · node.py 구독 · `test/test_localization.py`(3개). README 줄은 #21 과 충돌해 빼고 설명은 localization.py 에 둠.
결과: 브랜치 pytest 11/11, 합친 트리(`_merge16` + 이 패치) perception_survey 15/15, 열린 PR 13 개와 충돌 0. 노드 확인 `loc_gate_check.py` 2/2 통과(`loc_gate_r1` · `r2`). 1차(`loc_gate_fail1_toolread`)는 도구가 0.3 s 고정 대기로 결과를 한 박자 늦게 읽은 도구 문제 — 바뀔 때까지 기다리게 고침, 노드 코드 그대로.
