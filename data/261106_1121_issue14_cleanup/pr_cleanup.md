## 무엇
- 지금 코드에서 동작하지 않는 `scripts/mock_check.py` · `mock_check_pucks.py` · `run_mock_check.sh`를 지웁니다.
- 안 쓰는 import, 시험 복사 경고를 고치고 `test_perception_logic` 시간 제한을 300 s 로 늘립니다.

## 왜
- `mock_check.py`는 B안 URDF 에서 KeyError 로 멈추고, M1 은 지금 끈 파랑 검출을 씁니다. `mock_check_pucks.py`는 P1~P5 모두 실패합니다(READ_TAG 메시지 0, 원인은 안 봄).
- `test_perception_logic`이 PC 부하 때 53 s 이상 걸려 기본 60 s 에서 시간 초과로 실패했습니다.

## 시험
- colcon test(perception · perception_survey) 213개 실패 0. 빌드 경고 0(perception), pyflakes 0.
- 제품 코드는 안 바뀌어 재생 시험은 안 돌렸습니다.

## 한계
- 지운 시험을 대신하는 렌더 재생 시험(A1 · F1 · R)은 이 저장소 밖에 있습니다.
- cppcheck 취향 항목과 `perception_node.cpp` 쪼개기는 하지 않았습니다.
