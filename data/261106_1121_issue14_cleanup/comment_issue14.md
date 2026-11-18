인지 쪽에서 기준점을 파라미터로 만들었습니다(`arm.align_point`, 브랜치 `fix/perception-arm-align-point`). PR로 올립니다.

- 값: `[폭 비, 높이 비]`이고 기본 `[0.5, 0.5]`(화면 중심)이라 지금 동작과 같습니다. drive · manipulator는 고칠 것이 없습니다.
- (320, 400)은 옛 모델 값입니다. 지금 기준 모델(B안)로 계산하면 잡는 점(`gripper_roll_link` x 0.14 m)은 화면 아래 밖(v 552)입니다. 집게 축 위 점의 세로 위치는 거리마다 다릅니다(4 cm 앞 398 · 11 cm 앞 302 · 26 cm 앞 237 px, 640×480, f 283 px 가정, 모의 계산).
- 그래서 고정값을 넣지 않았습니다. 가로는 화면 중심이 맞아 drive FINE(x만 씀)은 그대로입니다. 세로는 옛 pick 보정만 쓰고, `toppick_tilt`는 `toppick.use_alignment: false`라 쓰지 않습니다.

남은 것
- [ ] 실제 값: 팔 해상도(#119)와 카메라 보정이 정해진 뒤 정렬 거리에서 재어 yaml에 넣습니다(실물, 인지).
- [ ] `TAG_VIEW` 자세: `manipulator.yaml`이 아직 TODO입니다(manipulator). 내려다볼 때 세로 위치(145 · 280 px)도 640×480 옛 모델 값이라 다시 봐야 합니다.
