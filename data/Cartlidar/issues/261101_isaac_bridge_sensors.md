## 요청 (인지 파트 → Isaac Sim 브리지, `workspace/tools/isaac_sim/waffle_bridge.py`)

팀 Isaac 브리지는 지금 `/clock` · `/isaac_joint_states` · `/imu/data` · `/e_stop` 만 냅니다. 카메라와 라이다가 없어 팀 Isaac 모의에서 인지 노드를 돌릴 수 없습니다. 인지는 지금 개인 렌더 · 재생 도구로만 모의 확인하고 있습니다. 아래 토픽을 내 주세요(perception.yaml · lidar.yaml 기본값).

| 센서 | 토픽 | 비고 |
|---|---|---|
| 앞 D435 | `/top_camera_front/color/image_raw` · `/top_camera_front/aligned_depth_to_color/image_raw` · `/top_camera_front/color/camera_info` | 깊이는 컬러와 같은 크기 · 같은 stamp, frame_id 는 URDF `top_camera_front_color_optical_frame` |
| 뒤 D435 | `/top_camera_rear/...` 같은 꼴 | |
| 팔 카메라 | `/arm_camera/image_raw` · `/arm_camera/camera_info` | frame_id `arm_camera_optical_frame` |
| 라이다 | `/scan_front` · `/scan_rear` | scan_merger 가 `/scan` 으로 합침 |

stamp 는 `/clock` 기준(use_sim_time). 영상 QoS 는 인지 기본 best effort(sensor_data)로 받습니다.
