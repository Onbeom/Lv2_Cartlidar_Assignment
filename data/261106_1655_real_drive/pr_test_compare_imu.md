## 무엇을
실기 ↔ Isaac 비교 시험(`test_model_compare.py`, #112)에 `imu_link` 자세 90° 차이를 알려진 차이로 등록했습니다(`KNOWN_FRAME_ROT`). 이 각도 그대로일 때만 통과하고, 차이가 없어지면 실패합니다.

## 왜
#162 로 실기 `imu_link` 가 yaw -90° 가 되어(OpenCR +y 가 차체 앞, 실물 확인) Isaac 과 90° 다릅니다. 실제 장착 차이라 표에 이유와 함께 적었습니다.

## 시험
- PC: `test_model_compare.py` 전체 통과(새 self-test `imu_yaw_diff_gone` 포함).

## 한계
- 시험 코드만 바꿨고 로봇 동작은 바뀌지 않습니다.
