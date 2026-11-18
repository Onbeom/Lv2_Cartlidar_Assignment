## 요청 (opencr_hardware twist_mux, 작성자 kanichong · LCY)
`config/twist_mux.yaml`의 locks에 위치 잃음 잠금을 넣어 주세요.
```yaml
      localization: {topic: /localization/lock, timeout: 0.5, priority: 20}   # nav(10)만 막고 drive(50) · teleop(100)은 통과
```

## 왜
- `localization` 패키지의 loc_monitor는 위치를 잃으면(OK · SUSPECT가 아니면) `/localization/lock`에 true를 10 Hz로 냅니다.
- 지금은 이 신호가 twist_mux에 연결되어 있지 않습니다. 그래서 위치를 잃은 동안에도 Nav2 명령이 그대로 나갑니다.
- 모의에서는 같은 일을 loc_monitor 게이트로 확인했습니다. K2e 시험에서 감지 0.17 s, 다시 찾은 오차 0.012 m로 통과했습니다.

## 주의
lock 토픽이 끊겨도 잠깁니다. 그래서 localization 패키지를 띄우지 않는 실행에서는 이 줄이 없는 설정을 써야 합니다. 시뮬레이션용 `twist_mux_no_obstacle.yaml`도 같습니다.
