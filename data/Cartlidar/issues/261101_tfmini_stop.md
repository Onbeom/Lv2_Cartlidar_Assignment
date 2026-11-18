## 요청 (인지 파트 → 구동 명령 담당: control · opencr_hardware)

인지의 TFmini 비상 정지 `/perception/obstacle_stop`(std_msgs/Bool, true = 멈춤)를 받는 노드가 dev(d04599b)에 없습니다(control · mission · opencr_hardware · bringup 에 사용처 0). 그래서 TFmini 가 앞 장애물을 봐도 바퀴 명령이 멈추지 않습니다.

- 인지 쪽: `tfmini_node`(/tfmini/range, 1층 정면 바닥 중앙) → `obstacle_stop_node` → `/perception/obstacle_stop`. 설정 `perception/config/lidar.yaml` 의 obstacle_stop 절.
- 부탁: 최종 바퀴 명령을 내는 쪽(예: twist_mux 잠금 입력, 또는 drive_node 명령 출력 직전)에서 이 값이 true 이거나 일정 시간 안 들어오면 속도 0. 인지가 모터 명령을 직접 내지 않는 방식입니다.
