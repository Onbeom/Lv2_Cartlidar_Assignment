## 무엇
- perception 에 `/perception/place_box/measure_front`(Trigger)를 더합니다. SEARCH_FRONT 모드에서 앞 D435 컬러 + camera_info K 로 place_box 를 돌려 상자 윗면 중심 · 요를 잽니다.
- 결과는 팔 측정과 같은 토픽(`/perception/place_box` · `/cells` · `/status`)에 냅니다. status 에 `"camera": "front"|"arm"` 을 넣었습니다.
- 사전값(`/perception/place_box/prior` 또는 파라미터)을 정하는 부분은 두 서비스가 같은 함수(`start_place_box`)를 씁니다. 앞 D435 는 요청 뒤에 찍힌 장만 씁니다.

## 왜
- 상자가 지도 자리에서 밀리거나 돌아 있을 때(가정 ±5 cm · ±45°, #225 · #226), 대기 자리에서 팔을 움직이지 않고 상자 자세를 먼저 재서 다가갈 방향 · 거리를 정하려는 것입니다.

## 시험
- perception 단위 시험 228/228 통과(최신 dev 47bc12b 위).
- 렌더 재생(모의, Isaac Sim D435 640 × 480, 상자 ±5 cm · ±45°, Nav2 ±0.05 m · ±0.1 rad, 퍽 0 · 2 · 6 개, 조명 2): 노드에 240 장을 흘려 240/240 이 중심 ≤ 15 mm · 요 ≤ 4° 였고, 99.6 % 가 ≤ 10 mm · ≤ 2° 였습니다. 최대 2.10 mm · 2.09° 입니다.

## 한계
- D435 컬러 왜곡은 보정하지 않습니다(K 만 씀). 실물 영상과 D435 장착 각 보정(요 ≤ 0.7° 필요, 계산)은 아직 확인하지 않았습니다.
- 이 서비스를 부르는 mission 쪽은 따로 냅니다.
