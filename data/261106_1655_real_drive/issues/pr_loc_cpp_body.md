## 무엇을
- 위치 추정 쪽 Python 노드 셋을 C++ 판으로 (perception 패키지): `scan_body_mask` · `loc_scan_merge` · `loc_monitor`(amcl 백엔드), 공통 `loc_core.hpp/.cpp` + `test_loc_core`. `localization.launch.py` 에 `loc_cpp`(기본 true, false = 예전 Python 판), carto 백엔드는 Python 판 그대로.
- (따로 커밋) `perception_survey`: TF 리스너를 조사 중에만 켬.

## 왜
- #265: Jetson 정지 상태 CPU 85 ~ 95 %. 위치 추정 Python 노드 셋이 코어 하나 기준 약 64 (loc_monitor 24 · scan_merge 21 · scan_body_mask 19), puck_survey 는 조사 안 할 때도 /tf 를 받아 약 28.

## 시험
- Jetson 실물 p7loc (11-15 22:48 ~ 23:45, 5 분 × 8 판, 판정 기준 11-14 검토 승인): 깨끗한 짝 두 쌍에서 셋 합 66.8 → 10.5 · 63.9 → 10.7, 전체 CPU −10.4 · −6.9 점, /scan_merged 10 Hz 유지, 인지 Hz −0.1.
- PC 재생 비교 (11-14 미로 bag, 같은 AMCL 에 두 판을 함께): 상태 19 번 · lock 4 번 · 다시 찾기 4 번 값 순서 같음 (시각 차 1.04 s 안), 차체 가림 스캔 100 % 같음, 합친 스캔 98.4 % 같음 (다른 장은 Python 이 뒤 스캔을 못 붙인 장).
- PC colcon test perception · localization 246 개 통과.
- perception_survey (PC, listen_test.py): start → finish(덜 된 조사라 실패, 리스너 그대로) → reset → start → reset 순서로 불러 `TransformListener.unregister()` 오류 없음, 다시 start 한 뒤 map → base_link TF 로 퍽 관측 40 개 받아들임 (거부 0, 처음과 같음).

## 한계
- perception_survey 는 다시 start 한 직후 TF 버퍼가 비어 있어, 움직이는 TF(/tf) 는 처음 몇 장이 'missing_tf' 로 빠질 수 있음 (시험은 정적 TF 라 0 장).
- 재생 bag 은 11-14 의 틀어진 녹화라 77 s 뒤 FAILED 로 끝남 → 긴 정상 구간 비교는 재녹화 bag 으로 다시 봄. 같은 바퀴 짝이 3 ↔ 4 하나뿐이라 짝 8 ↔ 2 는 다른 바퀴(README p7loc 2 차).
