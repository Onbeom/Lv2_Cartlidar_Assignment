## 요청 (인지 → manipulator)

장소 상자가 돌아가 있거나 밀려 있어도 3×3 칸에 놓을 수 있게, 인지가 눈앞 상자 칸 9개 좌표를 냅니다(PR fix/perception-place-box, /perception/place_box/cells, PoseArray, odom, 윗면 높이).

### 요청
1. place_<L>_<dx>_<dy>[_tilt] (#63) 가 고정 칸(approach_poses.yaml 의 fwd·lat) 대신 칸 좌표(odom 또는 base_link)를 받아 놓게 해 주세요. 예: pick_target 처럼 place_target x y z yaw.
2. 상자 위에서 내려다보는 팔 자세를 이름 있는 자세로 추가해 주세요(측정용). B URDF 기준: joint 0, −0.65, 0.00, 1.95, 0 → 카메라가 윗면 약 0.20 m 위에서 0.5° 기울기. joint4 1.95 는 한계(2.0) 가까이라 실물에서 확인이 필요합니다.
3. 칸 좌표는 상자 방향(요)을 따라 돌아 있습니다. 직육면체 퍽을 놓을 때 집게 방향을 상자 요에 맞출지 정해 주세요.
