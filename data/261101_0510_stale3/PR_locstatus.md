## 무엇을
- B0 측량(`perception_survey`)이 위치 추정 상태 `/localization/status`(std_msgs/String, 재위치 노드)를 받아, `OK` 가 아니면(SUSPECT · LOST · RECOVERING · CONFIRM · FAILED) 그 관측을 버리고 `rejected` 에 `localization_<상태>` 로 셉니다.
- 상태를 한 번도 못 받았거나 `localization_stale_s`(1 s) 넘게 안 오면 '모름'으로 보고 받습니다(재위치 노드 없이 띄우는 실행과 호환). 판단은 새 `localization.py`(ROS 없음, 시험 3개), 파라미터 `localization_topic` · `localization_stale_s`.

## 왜
측량은 `world_frame` ← 카메라 TF 로 퍽 관측을 칸에 넣습니다. SLAM 이 위치를 잃은 동안의 TF 로 넣으면 관측이 다른 칸으로 가거나 위치 평균을 흐립니다. SLAM 쪽 재위치 노드(위치 잃음 감지 · 다시 찾기)가 이 토픽을 냅니다(Reliable · TRANSIENT_LOCAL depth 1, 바뀔 때 + 10 Hz).

## 시험
- 단위: 이 브랜치 pytest 11/11(새 3개), 열린 인지 PR 들과 합친 트리 perception_survey 15/15. 열린 PR(#21~#29 · #32 · #33 · #42 · #44) head 와 병합 충돌 없음.
- 노드 확인(PC, 모의 · 도구): 실제 `puck_survey` + 가짜 TF · PuckArray · 상태 — 상태 없음 받음 → LOST · CONFIRM · SUSPECT 버림(이유별로 셈) → OK 받음 → 1.5 s 끊긴 뒤 받음, 2회 모두 같음.

## 한계
위치를 잃기 시작해 SUSPECT 가 나기 전(재위치 노드 기준 최대 3 s)에 들어간 관측은 버리지 못합니다. 측량은 칸마다 최소 2 관측 · 투표 충돌 시 완료 거부라 틀린 완료는 막히지만, 그 칸은 다시 봐야 할 수 있습니다. 재위치 노드는 아직 SLAM 쪽 개인 폴더에 있고 팀 launch 에는 없습니다.
