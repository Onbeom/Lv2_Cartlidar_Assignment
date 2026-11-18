## 무엇을
팔 영상의 촬영 시각 TF 가 아직 없으면 처리 주기를 버리지 않고 최대 25 ms(`arm_tf_wait_s`) 기다립니다. 그 주기에 쓸 팔 영상을 한 장으로 고정해, 기다리는 동안 더 새 장이 와도 TF 가 어긋나지 않게 했습니다. 25 ms 안에 안 오면 전처럼 다음 주기에 다시 봅니다.

## 왜
팔 영상이 관절 TF 보다 새것일 때가 많아 `/joint_states` 50 Hz 에서 주기를 약 30 % 잃었고, ARM_ALIGN 결과가 약 10 Hz 였습니다(`perception.yaml` 기준 15 Hz 필요).

## 시험
- PC: perception 빌드, 단위 시험 200 개 실패 0(`TfLookup.WaitTransformReturnsWhenArrived` 추가).
- Jetson, 관절 TF 는 50 Hz 흉내, 카메라 실물, 20 s: ARM_ALIGN 10.1 → 14.7 Hz, 나이 p99 59 ms. 탐색 앞뒤+ARM 9.1 → 14.7 Hz. 두 수정을 합친 판에서 ARM_ALIGN 3 회 반복 14.7~14.9 Hz, p99 58~60 ms.
- CPU 평균 최대 50.1 %, RAM 최대 2.12 GB, 최고 56.8 °C, 노드 죽음 0.

## 한계
- 실제 관절 `/joint_states` 를 쓰는 로봇에서는 아직 재지 않았습니다.
- 탐색+ARM 나이 p99 는 174~177 ms 로 그대로입니다(원인이 달라 이번 수정 범위 밖).
- 기다리는 동안 처리 주기(제어 콜백 그룹)가 최대 25 ms 멈춥니다.
