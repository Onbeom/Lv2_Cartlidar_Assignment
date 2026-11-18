# Nav2 연결 시제품 · 팀 반영 전 개인 검토용

(필수 · PC) 모의 시험으로 확인한 연결 노드/설정만 복사했다. 실물 및 Jetson 운용 완료가 아니며 팀 반영은 별도 허가를 받는다. 이 폴더에는 학습/모의 깊이/결과 로그를 넣지 않았다.

- `command_gate.py`: Nav2 속도 명령과 생성 시각·입력 heartbeat·TFmini 정지를 검사. 출력은 Nav2 가지의 `/cmd_vel_nav` TwistStamped.
- `navigation_coordinator.py`: `/navigation/goal_pose` 목표1회 유지, 경로 불가 시 취소·대기·재계획. 활성 목표 교체/취소 API는 아직 없다.
- `observation_monitor.py`: 깊이·CameraInfo·거리지도·TF의 유효성. 최신 촬영 TF 대기 시 검증 완료한 이전 관측이0.30초 이내인 동안만 유효성을 유지한다.
- `nav2.yaml`, `dynamic_v2.yaml`: 모의 설정. footprint는 새 모델 drive 충돌형상 XY 외곽 근사+20mm 여유, 속도/시간 문턱/높이는 실물 운용값 미확정.
- `navigation.launch.py`: PC 격리 도메인99·LOCALHOST, GPU flock·nvblox30분 상한. 실제 모터/펌웨어/드라이버/가상 odom을 실행하지 않는다. 이 launch 조합 자체는 아직 전체 실행 검증 전이며, 동일 핵심 노드를 직접 연결한 시험만 검증했다.

필요한 외부 입력: `/map` 정적 OccupancyGrid, map→odom→base_link와 카메라 TF, `/odom`, `/self_filter/output` 보정/좌표가 일치하는 깊이, `/self_filter/camera_info`, `/perception/obstacle_stop`. 외부 입력을 같은 격리 도메인에서 준비해야 한다. 현재 개인 자기필터와 SLAM 운용 launch를 하나로 묶은 실물 구성은 아직 없다.

**중요한 연결 경계:** 팀 mux는 drive/teleop이 nav보다 우선한다. nav 출력을0으로 만드는 것만으로 전체 정지가 보장되지 않는다. 최종 TFmini 정지 우선순위는 제어 담당이 모든 모드에 적용해야 한다. `/e_stop`에 중복 발행자를 추가하지 않는다. 요청 본문은 `Cartlidar/issues/261030_Nav2_명령_제어연결.md`.

내장 collision_monitor/velocity_smoother 결합·임의 우회경로의 즉시 교체·다중 목표/취소·발제 전체 월드·이동 VO/자기필터/실물센서/Jetson 검증은 남아 있다. 목표 재도달은 2D 운동학 모의이며 제동/실물 주행 결과가 아니다.
