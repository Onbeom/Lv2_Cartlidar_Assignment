## 무엇을
실물 LDS-03 앞 · 뒤 라이다 TF 요를 180° 고침: `robot.urdf.xacro` 의 `laser_front` yaw 0 → π, `laser_rear` π → 0 (`backend:=opencr` 일 때만). Isaac · mock 은 예전 값 그대로 (`imu_link` 와 같은 방식). `test_robot_description` 에 실물 yaw 확인, `test_model_compare` 에 알려진 각도 차이 180° 추가, `frames_tf.md` · `lidar.yaml` 주석 맞춤 (문서의 옛 브라켓 v1 자리도 v2 (±0.1318, 0, 0.0675) 로 맞춤).

## 왜
11-15 실물 상자 가림 시험에서 두 라이다 모두 roll π 는 맞고 요만 약 180° 틀림 (앞 174° · 뒤 165°). Isaac 브리지는 기준 모델 프레임으로 스캔을 만들어 이름만 `laser_*` 로 붙이므로, URDF 를 다 바꾸면 이번엔 시뮬레이션 스캔이 180° 돌아감 → 실물에만 적용. Closes #ISSUE_NO

## 시험
- PC: opencr_hardware colcon test 145 개 통과 (test_robot_description 7 · test_model_compare 10). 알려진 차이를 빼면 test_model_compare 가 laser_front · laser_rear 180° 로 실패하는 것 확인.
- xacro 펼침: opencr 앞 π · 뒤 0, isaac · mock 앞 0 · 뒤 π.
- 실물: 같은 변환을 쓴 점 보기 창으로 상자를 앞 · 옆 · 뒤에 대어 고친 쪽만 맞는 것을 사람이 확인.

## 한계
- 이 브랜치로 실물 bringup 을 띄워 보지는 않음. 합치기 전에 이 브랜치로 bringup 을 한 번 띄워, 앞에 상자를 두면 `/scan`(base_link) 의 +x 쪽에 점이 생기는지 확인함. 남은 몇 도는 미로 안에서 두 라이다 스캔 겹침으로 다시 맞춤.
- 라이다를 쓰는 곳은 모두 TF 를 거침 → 이 수정으로 함께 바로잡힘: scan_merger(`/scan`, drive_node 앞 · 뒤 막힘 검사가 씀) · scan_merge(AMCL 입력) · scan_body_mask(차체 가림) · loc_monitor · Cartographer(`*_filtered` 두 개) · Nav2 costmap · collision_monitor(`/scan_merged`). 원래 각도로 바로 쓰는 곳은 찾지 못함. obstacle_stop 은 TFmini 만 씀 (라이다 안 씀).
