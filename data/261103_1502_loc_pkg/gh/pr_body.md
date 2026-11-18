## 무엇
- 새 패키지 `localization`을 추가합니다. 들어 있는 것은 다음과 같습니다.
  - `loc_monitor`: 위치 잃음 감지와 다시 찾기. `/localization/status`와 `/localization/lock`을 냅니다.
  - 지도: `/map`, `/map_objects`
  - AMCL 입력: `/scan_merged`(차체 가림 지움, 움직임 보정)
  - AMCL 설정 `config/amcl.yaml`
- 기본은 AMCL이고, AMCL 노드는 주행(Nav2) 쪽 launch가 띄웁니다. `use_amcl_node:=true`를 주면 이 패키지가 AMCL도 띄웁니다. Cartographer 순수 위치 추정은 들어 있지만 기본에서는 꺼져 있습니다(`localization:=carto`).
- Closes #85

## 왜
- perception_survey는 `/localization/status`를 받는데, 지금 이 토픽을 내는 노드가 없습니다.
- AMCL을 kanichong 설정 그대로 쓰면 모의에서 제자리 회전 때 옆 오차가 커서 차체 여유(G3)를 통과하지 못했습니다. 옴니 모델 등 5개 값을 바꾼 설정을 넣었습니다.

## 시험 (Isaac 모의, B안)
- 단위 시험 3개 통과.
- K0(정상 주행, 목표 7개)과 K1(정지 중 0.20 m · 10° 강제 이동)을 팀 bringup `ekf.yaml`과 함께 돌렸습니다.
  - AMCL K1: 0.52 s 만에 감지, 다시 찾은 오차 0.0156 m로 통과.
  - AMCL K0: G1~G5 통과. 다만 잠깐의 SUSPECT가 2번 났습니다.
- 같은 판을 ICR 보정 EKF로 돌리면 SUSPECT 0, RMSE 0.0157 m로 통과합니다.
- AMCL 노드를 누가 띄울지 고친 launch 변경은 노드 목록 확인만 했습니다(모의 판 다시 돌리지 않음).

## 한계
- 지도, 출발 자세, 잔차 문턱 0.034 m는 모의 값입니다. 실물 SLAM 뒤에 바꿔야 합니다.
- 팀 EKF에 ICR 보정이 들어가기 전까지는 SUSPECT가 날 수 있습니다(별도 Issue).
- `fix/perception-survey-map-frame`(puck_survey world_frame → map)과 같이, 또는 바로 뒤에 합쳐야 합니다. 이 PR만 합치면 측량 관측이 0이 됩니다.
