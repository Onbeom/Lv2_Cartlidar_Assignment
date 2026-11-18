## 무엇을
실물 `imu_link` 에 yaw -90° 를 넣었습니다(`robot.urdf.xacro`, `backend:=opencr` 일 때만). Isaac · mock 은 그대로 0 입니다.

## 왜
아래 OpenCR 의 +y 가 차체 앞을 향합니다. 실물에서 앞쪽을 들면 가속도 y 가 + 로 커지는데 URDF 는 x 가 앞이었습니다(최대 y +4.13 m/s², x +0.04).

## 시험
- 실물 보드 원시 값: 앞 들기 y +, 반시계 회전 ωz + (합 +365°).
- xacro 펼침: opencr -90°, isaac · mock 0 확인.
- `test_robot_description` 에 yaw 검사 추가, 통과. opencr_hardware 시험은 `test_model_compare`(실물↔Isaac `imu_link` 90° 차이)만 실패.

## 한계
- ROS `/imu/data` 경로로 다시 재지는 않았습니다.
- EKF · `odom_icr` 은 ωz 만 쓰므로 주행 동작은 바뀌지 않을 것으로 봅니다(코드 확인).
