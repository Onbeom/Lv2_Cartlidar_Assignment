# TFmini 끊김 수정 · 2026-11-01 (사용자 '지금 수정해')

결함(10-29 찾음, 개인 수정 `261029_1408/tfmini_failsafe.patch`, 팀 미반영이었음): dev `tfmini_node` 는 시리얼이 끊기면 지금 시각의 +inf 를 50 Hz 로 계속 낸다.
`obstacle_stop` 은 이것을 새 측정으로 받아 끊김(stale) 정지를 하지 않고, 직전 유효 거리가 near_m(0.30 m)보다 멀었으면 무효를 '앞이 비었음'으로 본다 → 끊겨도 정지가 풀린 채로 남는다.

고침(브랜치 `fix/perception-tfmini-stall`, dev d04599b 위, 작업 트리 `~/git/Lv2_Cartlidar_Assignment_tfstall`, 패치 `tfstall.patch`): 끊긴 동안 내지 않음(tfmini_node.cpp 1줄 + 주석). 10-29 패치의 나머지(무효 거리 = 바로 정지 `invalid_stops`)는 동작 정책 변경이라 이번에 넣지 않음.

## 확인 (모의 · 도구 확인, 가짜 PTY 시리얼 → 실제 tfmini · obstacle_stop 실행 파일, 물리 센서 · 모터 없음)
순서: 켬 → 2.0 m 0.6 s → 시리얼 끊김 1.0 s → 2.0 m 0.6 s. `stall_check.py`.

| 항목 | 기준 | 예상 · 고치기 전(dev, `261101_0246_final/sb/build_branch`) | 예상 · 고친 뒤 | 결과 전 | 결과 뒤 |
|---|---|---|---|---|---|
| 끊김 중 정지 | 정지 | 안 됨 — +inf 를 '비었음'으로 받음 | 정지 | **안 됨**(재현) | **정지** 2/2 |
| 끊김 → 정지 지연 | ≤ 0.35 s (TTL 0.2 + 타이머 0.1 + 여유) | 없음 | 0.2~0.3 s | 없음 | **0.28 · 0.29 s** |
| 끊김 중 /tfmini/range 메시지 | 0 | 약 45개(50 Hz × 0.9 s) | 0 | 41개 | **0 · 0** |
| 켬 · 풀림 · 복구 | 정지 · 풂 · 풂 | 같음 | 같음 | **켬 때 정지 안 함**(예상과 다름: 센서가 한 번도 안 들어와도 +inf 를 내서 'no_return' 으로 풀림 — 같은 원인) · 풂 · 풂 | 정지 · 풂 · 풂 |
| 단위 시험 | 모두 통과 | — | dev 와 같은 개수(시험 추가 없음 — 드라이버는 시리얼 실행 파일이라 위 도구로 봄) || — | **128/128**(종료 코드 5 = dev perception_survey 시험 0개, #21 이 고침) |

결과 `before/summary.json`, `after/` · `after2/summary.json`(2026-11-01). 열린 PR 11개 head 와 병합 충돌 0. PR 본문 `PR_tfstall.md`.
