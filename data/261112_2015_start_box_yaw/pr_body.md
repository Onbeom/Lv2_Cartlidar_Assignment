**이 PR 을 합치면 #229(비스듬히 서기 `fix/mission-oblique-stand`, 담당 결정 대기)도 함께 합쳐집니다. #229 를 정한 뒤 합쳐 주세요.**

#226 제안 흐름의 mission 쪽입니다.

## 무엇
- `start_box.enabled`(기본 false)를 켜면 시작 박스 대기 자리에서 앞 D435 `measure_front` 로 상자를 잽니다(`measure_start`).
- 그 접근의 `turn_*` 는 남는 퍽 면 어긋남이 가장 작은 이름 각으로 바꿉니다. 범위는 `turn_range_deg` ∩ ±30° 이고, 실제 회전 중심 icr_x 둘레로 셉니다.
- `target_look:base_footprint:…` 는 잰 상자에서 낸 `target_look:odom:…` 으로 바꿉니다.
- 계산은 `mission/start_box.hpp`(gtest 7 개)에 두었습니다.
- 잰 돌림 |rot| ≥ 40°(`start_box.sign_warn_deg`) 이면 mission_log 에 '부호 모호' 경고를 남깁니다(멈추지 않음).
- `mission_planner/test/mock_robot.py`(kanichong 작성)를 고쳤습니다: measure_front 가짜(`--start_shift` · `--start_rot`), drive 와 같은 icr 둘레 회전, execute_skill 마다 events 기록.

## 왜
- 상자가 돌면 퍽 면과 J1 → 퍽 방향이 어긋납니다. #226 의 '기본각 + β' 는 방향이 반대입니다(계산, #226 댓글).
- 회전으로 맞출 수 있는 만큼 맞추고, 남는 각은 정책 집기가 받습니다.

## 시험 (모두 모의)
- S1 단위: 격자 풀이와 0.005° 안, 퍽 자리 1 mm 안입니다.
- S2 mock 미션 9 경우(β 0 · ±15 · ±30 · ±45, 밀림 ±5 cm): 모두 DONE 입니다. target_look 자리는 참 퍽과 0.07 mm 안이고, (회전각, 남는 각)은 따로 계산한 값과 0.1° 안입니다. 남는 각은 β ±30 불리한 쪽 23.7°, ±45 24.2 · 27.7° 입니다.
- 밀림 0.099 m > 0.08 이면 계획 줄 그대로 가고 DONE 입니다.
- S3: 꺼진 상태에서 보낸 skill 줄이 바꾸기 전과 같습니다.
- S4: colcon test mission · mission_planner 97 개가 모두 통과합니다.

## 한계
- 실물에서 확인하지 않았습니다. 켤 때 manipulator 도 `skill_enabled:=true` 로 띄워야 합니다.
- 정사각 윗면만으로 ±45 를 못 가르고, 틀리면 퍽 자리가 57 mm 틀립니다. target_look 의 match_distance 안인지는 확인하지 않았습니다(|rot| ≥ 40° 면 경고 기록).
- 옆(W · E) 접근 직육면체는 계획에 회전 줄이 없어 target_look 만 바뀝니다.
- |β| 약 27° 넘는 불리한 쪽은 남는 각이 20° 를 넘습니다(경고만 남깁니다). 면에 수직으로 다시 서는 방식은 따로 제안합니다.
