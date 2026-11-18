@kanichong 실물 TF(`opencr_hardware/urdf/robot.urdf.xacro`)와 Isaac 모델(`urdf_and_usd/isaac/urdf/waffle_4wd_loaded.urdf`)의 앞 · 뒤 라이다 자리가 24.3 mm 다릅니다. 실물에 어느 브래킷이 달려 있는지 확인을 부탁드립니다.

## 다른 점 (dev 07bd5b2, base_link 기준)
| | 실물 xacro (11-02) | Isaac 모델 (a1cea6e, LDS-03 브라켓 v2, 11-12) |
|---|---|---|
| laser_front | x 0.1089, z 0.0755 | x 0.1318, z 0.0675 |
| laser_rear | x −0.1089, z 0.0755 | x −0.1318, z 0.0675 |
- 앞뒤로 22.9 mm, 높이 8 mm 다릅니다. `opencr_hardware/test/test_model_compare.py` 가 dev 에서 이 차이(24.3 mm)로 실패합니다.

## 부탁
- 실물에 브라켓 v2 가 달려 있으면 실물 xacro 를 v2 값으로 맞추겠습니다(xacro 는 Onbeom 담당).
- 아직 v1 이면 Isaac 모델이 실물보다 앞서 있는 것이니, 언제 바꿀 예정인지 알려 주세요. 그동안 test_model_compare 에 알려진 차이로 넣어 두겠습니다.
- 라이다 자리가 틀리면 Cartographer · AMCL 의 스캔 정합과 TFmini · 장애물 정지 거리에 그대로 들어갑니다.
