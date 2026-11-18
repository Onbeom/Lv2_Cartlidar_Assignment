## 요청 (인지 파트 → mission)

dev(d04599b)의 mission_node(10-25)는 인지 출력 중 `/perception/result`(SEARCH_FRONT · SEARCH_REAR · ARM_ALIGN · READ_TAG)만 씁니다. 발제(주사위 기억 로봇)에 필요한 아래 인지 출력은 mission 에서 쓰지 않습니다. 모드 이름 표(`parseModeNames`)에도 READ_MARKER · READ_SIGNAL · READ_BARCODE 가 없습니다.

| 발제 단계 | 인지 출력 | mission 사용 |
|---|---|---|
| 퍽 12칸 측량(B0) | `/perception/survey/*` 서비스 · `/perception/puck_survey` | 없음 |
| 신호등 순서 | READ_SIGNAL → `/perception/signal`(SignalSequence) | 없음 |
| 바코드 | READ_BARCODE → `/perception/barcode`(Barcode) | 없음 |
| 장소 태그 | READ_MARKER → `/perception/task_marker`(TaskMarker) | 없음 |
| 주사위 칸 확인 | `/perception/check_dice`(CheckDice 서비스) | 없음 |

아래는 그중 퍽 측량 연결입니다.

| 이름 | 종류 | 내용 |
|---|---|---|
| `/perception/survey/start` · `/reset` · `/finish` | std_srvs/Trigger | 측량 시작 · 처음부터 · 끝내기(12칸이 모두 확정돼야 성공, 확정 뒤 고정) |
| `/perception/puck_survey` | std_msgs/String(JSON), Reliable · TRANSIENT_LOCAL · depth 1, 1 Hz | `slots[]`(box, side, x_mm, y_mm, color, shape, pose, id …), `complete`, `errors`. 좌표는 `map`(발제 원점, mm) |
| 완료 파일 | JSON | launch 인자 `survey_output`(기본 `~/.ros/puck_survey/puck_survey.json`) — finish 성공 때만 씀 |
| `/perception/pucks` | robot_interfaces/PuckArray | 영상마다 퍽 후보(base_link). 측량이 이것을 받아 칸에 쌓음 |

순서 예: 박스 앞 조사 자세로 이동 → start → (D435 로 보고, 팔 카메라로 퍽 윗면 보기) → 박스마다 반복 → finish 성공이면 `complete: true` JSON 사용, 실패면 `errors` 의 칸을 다시 보기.

근거: 인지 모의 판정(Isaac 렌더 재생, 판정 월드 6개 × 조명 4)에서 12칸 24/24회 완료, 틀린 완료 0. 실물은 아직입니다. launch 연결은 인지 PR #24, 확인 문턱은 #21 에 있습니다.
