## 무엇
camera_info의 K 형식(3×3 pinhole, 초점거리·주점 범위)과 영상 해상도를 검사해, 맞지 않으면 그 영상으로 퍽·마커·신호등·바코드 결과를 내지 않습니다(`camera_info_check.hpp`). 팔 카메라는 같은 가로세로 비면 기존처럼 비율 보정을 허용합니다.

## 왜
Isaac Sim 6.1 + Lyrical 기본 CameraInfo 발행에서 K가 한 칸씩 밀린 사례가 있고, D435 경로는 camera_info 해상도를 영상과 대조하지 않아 위치가 조용히 틀릴 수 있었습니다.

## 시험
perception 단위 시험 127/127 + 새 camera_info 시험 4/4 통과.

## 한계
실제 카메라·Isaac 장면 연결 시험은 아직입니다. frame_id 불일치는 경고만 합니다.
