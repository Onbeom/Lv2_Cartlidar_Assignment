# 인지 담당 범위의 SLAM·nvblox·Nav2 연결 가이드

2026-10-29. 팀 코드는 변경하지 않았다. 연결 설계이며 통합 실행 성공 보고가 아니다.

## 담당 범위와 선택

사용자 선호대로 **cuVSLAM의 odom + 라이다를 Cartographer에 넣는 구성**을 우선 검증한다. SLAM은 작업 범주이며 cuVSLAM·Cartographer와 별개의 세 번째 프로그램이 아니다. YOLO 지도학습과 이 위치추정 구성에 강화학습은 필요하지 않다.

| 구성 | 담당·우선순위 | 이유 |
|---|---|---|
| YOLO 및 태그/신호/바코드 인지 | 필수 · PC 개발/Jetson 운용 | 물체 속성·위치·시각·유효성 제공 |
| cuVSLAM odometry + Cartographer | 필수 · PC 모의/기록 검증 후 Jetson | 사용자 선호 구성, 연속 odom과 라이다 전역 지도 역할 분담 |
| nvblox | 필수 · PC/Jetson | 사용자 담당으로 확정, 깊이 장애물·거리장 연결 |
| Nav2·경로·미션 계획 | 시간 여유 시 우리 후속 범위 | 인지/위치추정 안정 후 경로·미션 통합, 실제 모터 제어는 다른 담당 |
| 팔/바퀴 모터 및 PPO 정책 | 현재 인지 범위 밖 | 인지 시험용 카메라 자세와 실제 제어 구현은 별개 |

## 연결 구조

```text
차체 고정 D435 좌우 IR → cuVSLAM → 연속 odom ─┐
앞/뒤 라이다 ───────────────────────────────┴→ Cartographer → map·지도 보정
D435 깊이 + 촬영 시각 TF → [필요 시 nvblox] → 거리장/장애물 비용지도
RGB + 깊이 + TF → YOLO 인지 → 종류·위치·신뢰도·시각
지도·위치·장애물 → 주행 담당 Nav2 → 제어 담당
```

cuVSLAM만으로 Cartographer 지도를 만드는 것이 아니라 **라이다 관측에 외부 odom을 추가**한다. 나쁜 odom이 들어가면 지도도 악화될 수 있으므로 기존 라이다 기준 구성과 같은 입력으로 비교한다.

## TF와 토픽의 소유자

(필수 · PC/Jetson) 같은 TF를 두 노드가 발행하지 않도록 한다. 현재 개인 비교 설정의 실제 기준은 base_footprint이며, 일반 설명의 base_link로 임의 변경하지 않는다.

```text
map --Cartographer--> odom --cuVSLAM--> base_footprint --URDF--> base_link --URDF--> 센서
```

| 설정 | 결합 후보 값 | 의미 |
|---|---|---|
| cuVSLAM enable_localization_n_mapping | false | odometry만 담당하는 후보 |
| cuVSLAM publish_map_to_odom_tf | false | 전역 보정은 Cartographer 소유 |
| cuVSLAM publish_odom_to_base_tf | true | 직접 연결에서는 cuVSLAM이 연속 TF 소유 |
| cuVSLAM base_frame | base_footprint | 기존 개인 URDF/비교 설정과 맞춤 |
| Cartographer use_odometry | true | 외부 odom 입력 사용 |
| Cartographer provide_odom_frame | false | odom→차체를 다시 만들지 않음 |
| Cartographer published_frame | odom | map→odom 발행 |
| Cartographer odom 입력 remap | /visual_slam/tracking/odometry | wheel odom과 명시적으로 구분 |

현재 개인 cuvslam_replay.launch.py는 단독 비교용으로 두 TF 발행이 모두 true다. **그대로 Cartographer와 동시 실행하면 안 된다.** 결합용 개인 launch를 별도로 만들고 검증해야 한다. 기존 bag의 wheel TF도 동시 발행되지 않도록 재생 토픽을 분리해야 한다.

(선택 · 권장, PC/Jetson) 추적 손실·미끄러짐 때문에 센서 융합이 필요하다고 측정되면 EKF를 비교한다. 이때 cuVSLAM의 odom→차체 TF도 끄고 EKF만 발행한다. wheel odom을 입력으로 받는 것은 모터 제어를 개발하는 일이 아니다. 현재는 EKF를 필수로 추가하지 않는다.

## PDF를 우리 로봇에 적용하는 범위

원문: [교안](../팀별%20로봇%20자율주행%202%20—%20nvblox·Nav2·예측%20카메라%20정책.pdf). 32쪽 전체 텍스트, 12·17쪽 연결도/수식 시각 확인.

교안 2·29쪽은 nvblox/Nav2/PPO가 제공 저장소에 구현되지 않은 확장 범위임을 명시한다. 24~26쪽 PPO는 별도 주행 정책 확장으로, 사용자의 인지 과제를 위해 따라 할 필요가 없다. 17쪽 EKF·AMCL 수식도 확장 예시다. Cartographer가 map→odom을 담당하는 구성에 AMCL을 동시에 켜지 않는다.

11~12쪽의 핵심은 **깊이+자세→nvblox**, **스테레오→cuVSLAM**의 분리다. nvblox는 깊이를 누적해 표면/거리장을 만들며, 주행 담당이 2D 비용지도 형태로 사용할 수 있다. YOLO의 검출 상자만으로 모든 장애물이 표현되는 것은 아니다.

(필수 · PC/Jetson, 설치 완료 후) nvblox를 연결할 때 우선 차체 고정 D435의 깊이와 CameraInfo, 촬영 시각 TF를 사용한다. RGB 팔 카메라에 깊이가 있다고 가정하지 않는다. 움직이는 팔 카메라 적분은 실제 깊이 센서와 촬영 시각 관절 TF가 있을 때 별도 확장한다.

교안의 voxel 0.025m, 적분15Hz, ESDF10Hz, mesh2Hz는 **시험 시작 예시**다. 0.05~0.65m 높이 범위와 robot_radius0.245m도 다른 로봇 값이므로 복사하지 않는다. 낮은 퍽/박스 검출을 요구한다면 높이 하한과 실제 깊이 관측 가능성을 먼저 확인한다. 성능 수치는 Jetson에서 YOLO+cuVSLAM과 동시 실행해 측정해야 한다.

교안 RealSense 그림의 투사기 켠 깊이/끈 IR 분리는 이 로봇에서도 확인할 항목이다. D435의 emitter 설정을 임의 변경하면 YOLO 깊이와 VO 입력에 함께 영향을 줄 수 있다. 기존 기록상 D435이며 IMU가 있는 D435i로 가정하지 않는다. 최신 cuVSLAM 문서는 tracking_mode를 사용하므로 교안의 enable_imu_fusion 이름을 그대로 복사하지 않는다.

## 검증과 인계 순서

1. (필수 · PC) 기존 모의 기록으로 결합용 launch와 TF 소유권/시간/토픽을 확인한다. 먼저 예상·기준을 STATUS에 기록한다. 과거 단독 cuVSLAM 결과가 좋지 않았으므로 결합 성공을 미리 단정하지 않는다.
2. (필수 · PC) 같은 입력에서 기존 Cartographer 기준과 cuVSLAM odom 결합 결과의 궤적·복귀오차·지도벽오차·추적손실을 비교한다. 같은 결정적 모의 경로 반복은 독립 실험으로 세지 않는다.
3. (필수 · Jetson, 사람 장비 단계) 실제 스테레오 동기/보정, 팔 가림, IR 투사기, GPU 부하를 확인하고 실물 bag을 받는다. 모의 통과는 실물 통과가 아니다.
4. (필수 · PC/Jetson, 설치 완료 후) nvblox 깊이 장애물 지도를 연결한다. 촬영 시각 TF가 없으면 현재 TF로 대신 적분하지 않고 무효 상태를 보고한다.
5. (필수 · PC/Jetson) 주행 담당에 frame_id, timestamp, 지도/odom 토픽, 유효 상태와 지연/추적손실 신호를 인계한다. 속도 제한·정지 동작·경로 계획은 주행/제어 담당이 구현한다.

읽기 확인 명령은 노드가 실행 중인 컴퓨터에서 수행한다. 아래는 실행 성공을 주장하는 명령 목록이 아니라 연결 점검용이다.

(필수 · PC 또는 Jetson의 ROS 터미널) 실제 odom의 frame과 시각 확인.
```bash
ros2 topic echo /visual_slam/tracking/odometry --once
```

(필수 · 같은 컴퓨터) 기존 기준 프레임 사이 연결 확인.
```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

(필수 · 같은 컴퓨터) Cartographer 전역 보정 확인.
```bash
ros2 run tf2_ros tf2_echo map odom
```

## 현재 한계와 제출

이 문서는 연결 가이드다. 이후 개인 결합 launch의 짧은 모의 연결 시험은 수행했으며, 긴 기록 정확도 비교는 진행 중이다. nvblox/Nav2 통합은 아직 수행하지 않았다. 팀 제출은 검증한 인지 코드·운용 YOLO 모델/클래스 설정·선택한 SLAM launch/설정/운용 지도만 후보로 삼는다. 렌더·학습사진·중간 모델·비교 bag은 개인 폴더에 보관한다. 팀 변경은 사용자 허가 후 fix/ 브랜치에서 하며 커밋·push는 사용자가 한다.

근거(2026-10-29 확인):
- [cuVSLAM 공식 매개변수·Odometry](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)
- [Cartographer 공식 외부 odom/TF 설정](https://google-cartographer-ros.readthedocs.io/en/stable/configuration.html)
- [nvblox 공식 역할](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)
- [Nav2 상태 추정 역할](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)
- 개인 기존 설정: 261027_0953/slam/cuvslam_replay.launch.py, carto_c6_2lidar.lua. 과거 모의 결과는 .claude/STATUS_yolo.md 참고.


최신 범위 정정: nvblox는 우리 담당으로 확정했다. Nav2·경로 추정·mission planning도 시간이 남으면 진행한다. 본문 초기 선택/타담당 표현보다 이 지시가 우선한다. 상세 현재/미구현 구조는 2026-10-29_인지_노드토픽_구성.md 참고.

현재 PC nvblox 실행 패키지는 사용자 설치 후 5.0.0 확인 완료. 실제 입력 연결 검증을 이어간다.
