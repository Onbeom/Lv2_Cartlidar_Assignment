## 무엇을
`sensors.launch.py` 의 팔 카메라 노드에만 Fast DDS 설정(`perception/config/fastdds_arm_camera.xml`)을 줍니다. 공유 메모리 세그먼트 32 MB, UDP 는 유지합니다. QoS 는 그대로입니다.

## 왜
1280×720 bgr8 한 장(2.76 MB)이 기본 공유 메모리(0.5 MB)보다 커서, 인지가 쓰는 best effort 구독에서 장이 버려졌습니다. 드라이버는 15.3 Hz 인데 구독은 11 Hz(구독 5 개 동시면 5.4 Hz), 최대 1.5 s 끊김이 났습니다.

## 시험 (실물 Jetson, f07edb1 + 이 수정, control 코드 끔)
- 팔 best effort: 구독 5 개 동시 15.3 Hz, 간격 최대 73 ms(전 5.4 Hz, 1502 ms). 팔 혼자 구독 하나도 15.3 Hz.
- D435 영향 없음: 컬러 · 깊이 reliable 29.9~30.0 Hz, best effort 28.6~29.7 Hz. launch 유지.

## 한계
- launch 복사본과 `~/real_drive` 의 XML 로 시험했습니다. 설치 경로로는 병합 뒤 다시 확인합니다.
- PC 등 다른 컴퓨터에서 이 토픽을 받는 경우는 시험하지 않았습니다.
- 팔 카메라 드라이버 주기가 요청 30 fps 의 절반(15.3 Hz)인 원인은 이 수정과 별개이며 아직 모릅니다.
