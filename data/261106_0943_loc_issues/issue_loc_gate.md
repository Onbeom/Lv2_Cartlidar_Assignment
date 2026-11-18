## 요청 (control `drive_node`)
`drive_node` 가 Nav2 목표를 보낼 때 `/localization/status` 를 보게 해 주세요. 지금 `control` · `mission` 코드에는 이 토픽을 읽는 곳이 없습니다.

## 규칙
1. 목표를 보내기 전에 `/localization/status == OK` 를 기다립니다. 모의 도구에서는 최대 60 s 를 기다렸습니다.
2. 주행 중 OK 가 아니었다가 OK 로 돌아오면, 지금 목표를 취소하고 같은 목표를 다시 보냅니다.

토픽: `std_msgs/String`, Reliable · TransientLocal, 10 Hz. 값은 `OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED` 입니다.

## 왜
- 위치를 다시 찾으면 추정 자세가 뜁니다. 그 전 자세로 만든 경로를 그대로 쫓으면 안 됩니다.
- twist_mux 잠금(#93)은 위치를 잃은 동안 속도 명령만 막습니다. 잠금이 풀린 뒤 낡은 경로를 쫓는 문제는 막지 못합니다.
- Isaac 모의 강제 이동 시험(Cartographer 구성, 실물 값 아님): 이 규칙 없이(K2d) 낡은 경로를 쫓아 실패, 규칙을 넣고(K2e) 통과(감지 0.17 s, 다시 찾은 오차 0.012 m, 기다림 2.98 s 한 번).
- SUSPECT 를 OK 가 아닌 것으로 칠지는 주행 쪽에서 정해 주세요. 모의 규칙은 OK 만 통과로 보았습니다.

## 참고
- 구현 예: 인계 묶음 `reference/sim_tools/run_goals.py` 의 `--loc_gate`, 규칙 설명은 `01_interface.md`.
- 잠금 중 Nav2 progress checker 가 목표를 중단시키면 mission 재시도(#93 댓글의 120 s × 3)와도 엮입니다. 실물 확인은 #111 입니다.

관련: #85 #93 #111
