조립품이 나오기 전까지 미션 · 통합 쪽 남은 일 가운데 아래 항목을 제가 맡습니다. 겹치지 않게 범위를 적어 둡니다.

## 맡는 일

| 작업 | 지금 상태 | 끝나는 기준 |
|---|---|---|
| 태그 둘레 추적 손실 측정 (#171) | 팔이 멈춘 클립으로만 잼 | 팔 + Arducam 으로 팔이 움직일 때 놓치는 비율을 재서 기록 |
| 재검증 기록 (발제 06 쪽 이동 후 검증 8 점) | 칸 판정 · 장소 태그만 `mission_log.jsonl` 에 남음. 퍽 ID 와 전체 중복/누락 판정 코드 없음 | 장소별 새 관측 퍽 ID · 위치와 12 개 전체 중복/누락 판정이 기록에 남음. 단위 시험 · mock 확인 |
| survey → planner 한 번 실행 | bringup 이 `pucks_file` 로 연결했지만 Gazebo 시험은 survey 를 끄고 mock 에 survey 가 없음 | 측량 결과 파일로 계획까지 한 번의 실행으로 이어지는지 확인 |
| #154 팔 카메라 시야 자세 | READY · BOX_VIEW 에서 퍽이 화면에 안 들어옴 | 정밀 정렬 · 퍽 조사 · 장소 상자 재기에 쓸 자세 제안과 모의 확인 |
| 이동 전후 이미지 저장 (발제 06 쪽 기록 항목) | mission · planner · bringup 에 저장 코드 없음 | 사건 단위로 일정 간격 몇 장 저장 (이동 전 = 시작 박스 조사, 이동 후 = 장소 재검증) |
| mission 설정 정리 | 옛 태그 미션표가 남아 있고 bringup 기본값이 `mission_source=tag` | `mission_table.yaml` · `mission.yaml:44 coarse_distance` · `mission.yaml:50 diagnostics.timeout` · `mission_node.cpp:358` 정리, 기본 실행이 plan 미션 |

`mission.yaml:50` 의 3.0 변경은 실물에서 `/diagnostics` 를 확인한 뒤에 합니다. `coarse_distance` 값은 drive · 팔 길이와 맞춰서 정합니다.

## 맡지 않는 일

problem.md · signals.json · placement.csv · 할당 정답표 · 검증 기록, 복귀 3 초 정지 확인, #160 은 이 Issue 범위가 아닙니다.

## 부탁

- @kanichong 위 mission 파일을 고치고 계시면 알려 주세요. 같은 파일을 동시에 고치지 않게 맞추겠습니다.
- @developlcy-oss #154 자세는 `manipulator.yaml` 을 건드리게 되어, 값을 올리기 전에 먼저 보여 드리겠습니다.

작업은 항목별 `fix/` 브랜치와 PR 로 올리고, PR 마다 다른 분께 리뷰를 부탁드리겠습니다.
