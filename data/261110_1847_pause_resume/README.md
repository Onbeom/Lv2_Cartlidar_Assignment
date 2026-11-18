# 4번 이동 전후 사진 — PAUSED 재개 · 정지 뒤 새 시작 (모의, PC)

- 목적: PR #186(`fix/mission-snapshot`, b417e10)에서 코드 구조와 단위 시험으로만 확인한 두 가지를 실제 실행으로 확인한다.
  - PAUSED 재개 = 같은 에피소드 폴더, 사진은 덮어쓰지 않음
  - 정지 뒤 새 시작 = 새 에피소드 폴더
- 구성: `261110_1003_snapshot/run_mock.sh` 와 같다(그 폴더의 설치본 · 가짜 노드를 씀). 여기에 `pause_ctl.py` 를 더했다.
  - 기록 jsonl 에 snapshot 줄이 나오고 1.0 s 뒤(3 장 중 둘째 장 즈음)에 끊는다.
  - pause: `/e_stop` true → PAUSED 확인 → 3 s → false → `/mission/start`
  - stop: `/mission/stop` → IDLE 확인 → `/mission/start`
- 실행(PC, 모의): `./run_pause.sh R1 pause:before_r1_B2 pause:after_` · `./run_pause.sh R2 stop:before_r1_B3`
- 판정 근거: `run_<이름>/mission_log/*/images`, `mission_log.jsonl`, `ctl.log`, `mock.log`, `perception.log`.

| # | 시험 | 예상 | 결과 |
|---|---|---|---|
| R1 | survey B2 사진 도중 · 첫 verify 사진 도중에 각각 PAUSED → 재개 | DONE. 에피소드 폴더 1 개. PAUSED 2 번 확인. 끊긴 단계는 다시 하므로 사진 요청 번호가 2 개 늘어 01~11, snapshot 줄 11. 끊긴 요청도 perception 은 계속 찍음(pause 가 모드를 끄지 않음) → 1~3 장. 같은 이름 파일 없음(`_2` 덧붙임 0). verify_ids passed | 통과(예상 한 곳 틀림, 아래). DONE(78.3 s), 폴더 `261110_1848` 하나. PAUSED 2 번 확인(`survey_view 2`, `verify L2`), 둘 다 같은 단계에서 재개. 사진 요청 01~11, 33 장: 끊긴 `02_before_r1_B2` 도 3 장이고 재개 뒤 `03_before_r1_B2` 3 장, `08_after_L2` · `09_after_L2` 각 3 장. 같은 이름 덧붙임(`_k_2.jpg`) 0, snapshot 줄 11. **verify_ids 는 passed false(read 0)** — PAUSED 때문이 아님: 같은 구성 M1 · M2(PAUSED 없음)도 read 0 · false. 이 mock 은 verify 동안 퍽 ID 를 주지 않음 → 예상이 틀렸고 이 구성으로는 판정 못 함 |
| R2 | survey B3 사진 도중 `/mission/stop` → `/mission/start` | 에피소드 폴더 2 개(같은 분이면 둘째는 `_2`). 첫 폴더는 01~03 요청과 finish IDLE. 둘째 폴더는 01~09 · 27 장 · DONE. mock DONE | 통과. 첫 폴더 `261110_1850`: 01~03 각 3 장(정지 뒤에도 끊긴 03 은 3 장 다 씀), finish IDLE "stopped by user". 둘째 `261110_1850_2`: 01~09 · 27 장, DONE, 계획기 사본 · 등록표 정상. mock DONE(76.7 s) |

## 알게 된 것
- PAUSED 는 팔 카메라 모드를 끄지 않는다. 그래서 끊긴 사진 요청도 3 장을 다 찍고, 재개 때 같은 사건을 새 번호로 3 장 더 찍는다. 덮어쓰지는 않지만 같은 사건 사진이 두 묶음이 된다. 이게 맞는 동작으로 본다: 재개 뒤가 실제로 판정한 장면이다.
- `/mission/stop` 뒤에도 이미 받은 요청은 끝까지 쓴다(3 장).
- 이 구성의 verify_ids 는 mock 이 verify 동안 퍽 ID 를 주지 않아 늘 read 0 이다. verify 기록 자체는 `261110_0205_verify_record`(ok · dup · unread)에서 확인했다.
- 끝난 뒤 남은 시험 프로세스 없음(ps 로 확인).
