# ① 문제 패키지

| 파일 | 내용 |
|---|---|
| `problem.md` | 신호 · 바코드 · 장소 그룹, 퍽 12 개 시작 좌표 |
| `signals.json` | R/Y/G 12 회 순서 · 색별 횟수 · 바코드 · L1 ~ L3 그룹 |
| `placement.csv` | 퍽 ID · 색 · 모양 · 시작 박스 · 좌표 (mm, 발제 좌표) |
| `allocation_answers.md` · `.json` | 유효한 전체 퍽 할당 정답 전부 (L1 ~ L3 그룹 · 목적지별 퍽 ID) |
| `verification.md` | 발제 04 쪽 운영자 검증 ① ~ ④ 결과 |
| `review.md` | 다른 팀 검토 기록 (작성 필요) |

## 만든 방법 (재현)

```bash
cd workspace/ros2_ws/src/mission_planner
python3 -m mission_planner.submission --problem config/problem_q1_example.yaml --out ../../../../project2/problem
```

설치된 환경에서는 `ros2 run mission_planner mission_submit --problem <문제>.yaml --out project2/problem` 와 같다.
규칙 위반이면 `verification.md` 에 실패 원인을 쓰고 종료 코드 2 로 끝난다.

## 주의 — 지금 들어 있는 문제

지금 파일은 **발제 05 쪽 공개 연습 문제 Q1** (신호 `RGRRGRGGYGYG` · 바코드 `P2-01`) 을 쓴
`mission_planner/config/problem_q1_example.yaml` 로 만들었다. 퍽 색 · 모양 · 자리는 그 파일의 설명용 값이다.

팀이 낸 경기 문제 (또는 공식 경기에서 쓴 문제) 가 따로 있으면, 그 신호 · 바코드 · placement 로 다시 만들어 덮어쓸 것:

```bash
python3 -m mission_planner.submission --placement <placement.csv> --signal <12 글자> --barcode <P2-0x> --out ../../../../project2/problem
```

주사위 눈 배치 좌표 (장소별) 는 이 도구가 내지 않는다. 계획기 출력 `results/sim_plan_q1/plan.json` 의 놓기 칸 좌표를 참고하거나 따로 적는다.
