## 요청 (인지 → mission)

장소 상자가 돌아가 있거나 발제 좌표에서 밀려 있으면, 지금 방식(approach_poses.yaml 의 지도 좌표 칸 + AMCL 자리)으로는 3×3 무늬와 verify 판정이 둘 다 틀어집니다. 인지에 눈앞 상자 윗면을 직접 재는 서비스를 넣었습니다(PR fix/perception-place-box).

### 인지가 주는 것
- /perception/place_box/measure (std_srvs/Trigger): 팔 카메라가 켜진 모드(READ_TAG 또는 ARM_ALIGN)에서 부르면 팔 영상 3장으로 상자 윗면을 찾습니다.
- 결과(모두 latched, odom 기준):
  - /perception/place_box (PoseStamped): 윗면 중심, 요
  - /perception/place_box/cells (PoseArray): 칸 9개, 행 우선. dx −1→1 바깥 반복, dy −1→1 안쪽 반복. 칸 간격 0.06 m
  - /perception/place_box/status (String JSON): ok, frames, x, y, yaw, spread. 실패하면 ok false
- 모의(Isaac 렌더 120장, 상자 요 ±30°·옆 ±5 cm·놓인 퍽 0~9개): 중심 최대 0.68 mm, 요 0.37°, 칸 0.78 mm. 노드 재생 20/20, 한 번에 약 1.5 s. 실물은 미검증.

### 요청
1. place 단계 순서: approach(TFmini) 뒤 팔을 상자 위에서 내려다보는 자세로 → set_mode READ_TAG → /perception/place_box/measure → status ok 를 기다림 → cells 로 놓기.
   - B URDF 기준 자세: joint 0, −0.65, 0.00, 1.95, 0 (카메라가 윗면 약 0.20 m 위에서 거의 수직). TAG_VIEW 처럼 이름 있는 자세로 manipulator 와 맞춰 주세요.
2. place_<L>_<dx>_<dy> 는 고정 칸 이름 대신 cells 의 해당 칸 좌표를 쓰게(manipulator 쪽 Issue 와 같이).
3. verify 의 check_dice 칸 중심도 지도 좌표 대신 같은 cells 로(frame_id odom). 지금은 상자가 밀리면 맞게 놓아도 OFFSET 이 납니다.
4. 상자 방향은 90°마다 같아서, 로봇 기준 ±45° 안을 고릅니다. 주사위 무늬 '위' 방향이 지도 기준인지 상자 기준인지 발제 확인이 필요합니다.
