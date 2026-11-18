# 노드 연결 뒤 단위 시험 · 회귀 · 서비스 응답 확인 (2026-10-23, 모의 · 영상 없음, 실물 아님)

코드: 팀 저장소 `feat/perception-pucks` 커밋 전 작업본(`source_sha.txt`).
바뀐 것: 퍽 태그 16 mm·번호 0~11만, 장소 태그 tag36h11 ID 12~14·48 mm, 끝면 모양 판별(대각 18 mm 네 점), 신호등 영상 후보·자리 고정(signal_light.hpp)·노드 연결(/perception/signal, READ_SIGNAL=32), 주사위 확인 서비스(/perception/check_dice).

| 확인 | 결과 |
|---|---|
| 단위 시험 전체 (`gtest.log`) | 56개 통과 (U1~U3 50 + 끝면 모양 2 + 신호등 후보·자리 고정 4) |
| 회귀 M1~M5 (`mock_check_261023_1915_m_regress3`) | 모두 통과 |
| 서비스 응답 (`smoke_services.txt`) | set_mode 32 수락·앞뒤 D435 구독, 64 거부, check_dice 는 퍽 목록 없음 → observed=false |
| 커밋 나눔 확인 | ⑨ 상태(태그·모양) 빌드·단위 52개 통과, ⑩ 상태(인터페이스) 빌드 성공 — 저장소 밖 scratchpad 에서 |

아직 안 한 것: 신호등 영상 검출 값(v_min 200 · s_min 90 등)은 가정값 → R1(Isaac Sim 렌더)에서 정함. 주사위 확인의 실제 영상 판정은 R6.
