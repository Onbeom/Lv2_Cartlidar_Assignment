## 요청 요약
상자가 지도 자리에서 밀리거나 돌아 있어도(가정: 위치 ±5 cm, 회전 ±45°, #225 · #226) 장소 상자를 재고 볼 수 있도록 mission 에 '비스듬히 서기' 흐름을 넣었습니다. 브랜치 `fix/mission-oblique-stand` 를 봐 주시고 합칠지 정해 주세요. 기본값은 꺼짐(`oblique.enabled: false`)이라 켜지 않으면 지금 동작과 같습니다.

## 바뀌는 흐름 (켰을 때)
- `measure_box <L>` 이 `approach_box` 앞(대기 자리)으로 옵니다. 순서는 다음과 같습니다.
  1. 앞 D435 로 상자 재기(`/perception/place_box/measure_front`)
  2. 차체 축이 잰 중심을 지나게 제자리 회전(`/drive/navigate` 같은 x, y · yaw + φ)
  3. `align BOX` 로 보기 자리(앞 끝 → 가장 가까운 상자 점 0.35 m)까지 다가감
  4. `box_look` 0.44 m · 50°
  5. 팔 place_box
- verify 앞 접근은 `approach_view <L>`(보기 자리까지), verify 의 `box_look` 은 0.40 m · 55° 입니다.
- `box_look:<frame>:<x>:<y>:<z>:<거리>:<각>`(7 필드)이 필요합니다. 팔 쪽 변경은 manipulator Issue 에 따로 요청합니다.
- 로그: `mission_log.jsonl` 에 `oblique` 줄(β, 회전각, 목표 거리, tfmini / tfmini_wide).

## 시험 (모의)
- mission 단위 시험 27 개 통과(새 `test_oblique` 4 개 포함).
- mock 미션 6 장면 모두 DONE.
  - 꺼짐은 지금 dev 와 단계 · 주행 · 팔 기록이 같습니다.
  - 상자 0° · 20° · 40° · −45° 에서 회전각 · 목표 거리가 계산식과 맞고, 30° 를 넘으면 경고와 기록(`tfmini_wide`)만 남깁니다(방식은 같은 TFmini).
- 보기 자세 두 개는 Isaac Sim 렌더 판정 통과(모의): place_box 224/224, 태그 98.9 %. perception 앞 D435 측정은 240/240.

## 한계 · 남은 것
- |β| > 30° 는 TFmini 빔이 상자 모서리에 걸릴 수 있습니다. drive 에 odom 직진이 생기기 전(#225)까지는 TFmini 로 대신하고 경고를 남깁니다.
- 놓기 때 가까이 붙기(β 에 따라 36 ~ 117 mm, #225)와 시작 박스 survey 는 넣지 않았습니다.
- 실물 · Isaac 미션 시험은 하지 않았습니다.
