## 요청 (인지 파트 → bringup · 통합)

`bringup/launch/bringup.launch.py`(dev d04599b)는 OpenCR · ros2_control · 인지 · 판단 · 제어 · EKF 를 띄우지만, 인지가 받는 센서의 드라이버는 띄우지 않습니다. 실물 통합 때 아래 드라이버가 같이 떠야 인지가 입력을 받습니다(토픽은 perception.yaml · lidar.yaml 기본값).

| 센서 | 인지가 받는 토픽 | 비고 |
|---|---|---|
| 앞 D435 | `/top_camera_front/color/image_raw` · `/top_camera_front/aligned_depth_to_color/image_raw` · `/top_camera_front/color/camera_info` | realsense2_camera, 깊이를 컬러에 맞춤(align_depth), frame_id 는 URDF `top_camera_front_color_optical_frame` 과 같게 |
| 뒤 D435 | `/top_camera_rear/...` 같은 꼴 | 시리얼 번호로 두 대 구분 |
| 팔 Arducam | `/arm_camera/image_raw` · `/arm_camera/camera_info` | UVC 드라이버, frame_id `arm_camera_optical_frame`, 보정값(camera_info) 필요 |
| LDS 라이다 앞 · 뒤 | `/scan_front` · `/scan_rear` | scan_merger 가 `/scan` 으로 합침, frame_id 는 URDF 라이다 프레임 |
| TFmini | (시리얼 `/dev/tfmini`) | 인지 `tfmini_node` 가 읽음 — bringup 의 control_container 에 이미 있음 |

카메라 · 라이다를 끄고 띄우는 인자(예: `use_sensors:=false`)가 있으면 모의(Isaac)와 같이 쓸 수 있습니다.
