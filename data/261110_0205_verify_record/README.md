# 261110_0205 재검증 기록 (발제 06 쪽 이동 후 검증 8 점)

팀 저장소 작업 폴더 `~/git/Lv2_Cartlidar_Assignment_verify` (브랜치 `fix/mission-verify-record`, origin/dev e7cdde7 에서).
빌드 · 시험 결과물은 이 폴더(`build/ install/ log/`)에 둔다. `verify_pucks_mock.py` · `run_mock.sh` 는 확인용이라 팀 저장소에 넣지 않는다.

## 무엇을 바꿨나
- `mission/include/mission/verify_record.hpp` (새 파일, ROS 없음): verify 동안 받은 퍽 목록을 칸에 넣어 (칸, 태그 ID) 별로 쌓고,
  모든 장소 기록으로 ID 중복 · 누락 · 기대 밖 ID 를 판정.
- `mission_node.cpp`: plan 모드에서 `/perception/pucks` 구독. verify 자세 · 모드가 선 뒤부터 check_dice 요청 직전까지 쌓아
  verify 줄에 `"ids": {frame, frames, pucks: [{cell, id, seen, xy, map, offset}]}` 를 붙이고, 마지막 verify 뒤 `"verify_ids"` 줄.
- `CheckDice.srv` · `dice_check.hpp` 는 바꾸지 않음(robot_interfaces 동결, Jetson 클린 빌드 불필요).
- `mission_plan.yaml` verify 에 `record_ids · expected_ids · id_min_seen · region_margin`.

## 시험 결과

| 시험 | 종류 | 예상 (값의 뜻 · 예상과 이유) | 결과 |
|---|---|---|---|
| 단위 `test_verify_record` 7 개 | 모의(계산만) | 통과 수: 7/7. 12 개 정상 · 장소 간 중복 · 태그 못 읽음 · 한 장 오독 · 범위/높이 밖 · 기대 밖 ID · 재시작 초기화를 각각 만든 자료라 | 7/7 통과 (`test.txt`) |
| mock 미션 ok | 모의(mock_robot + 보조 노드) | verify 줄 3 개의 ids.pucks 수 = 그 장소 눈 수(합 12), 모두 id ≥ 0. verify_ids read 12 · unread 0 · passed true. 칸 밖 · 바닥 퍽은 기록 없음. 미션 DONE | 6 · 4 · 2 개(L3 · L2 · L1), ID 0~11 모두, 칸 중심 거리 0.1~1.4 mm, seen 16/50 장(3 장에 1 번 읽힘). 칸 밖 · 바닥 퍽 기록 없음. 칸 좌표계 odom(장소 상자 잰 값)에서도 지도 위치 맞음. verify_ids passed true. DONE 51.6 s (`mock_ok/`) |
| mock 미션 dup | 모의 | verify_ids duplicate = [두 장소에 낸 같은 ID], missing = [빠진 ID 하나], passed false. stop_on_fail false 라 미션 DONE | duplicate [0], missing [6], passed false, DONE (`mock_dup/`) |
| mock 미션 unread | 모의 | 세 번째 장소 마지막 칸 id −1 기록, verify_ids unread 1 · missing 1 개 · passed false. 미션 DONE | L1 칸 8 id −1 (seen 50), unread 1, missing [11], passed false, DONE (`mock_unread/`) |

시험 안 한 것: `verify.stop_on_fail: true` 에서 ID 판정 실패로 단계가 실패하는지(코드는 dice · 장소 태그와 같은 방식), 실물 · Gazebo 영상의 태그 판독.

## 알게 된 문제 (팔 카메라 시야, 계산만)
box_look 첫 후보(90° 바로 아래, 카메라 → 상자 윗면 0.20 m)에서 퍽 윗면 태그(윗면 z 0.13)까지 0.14 m.
Arducam 실측 fy 865.6 · cy 359.6 → 세로 반 시야 359.6 / 865.6 × 0.14 = **0.058 m**. 3×3 바깥 줄 태그 중심은 ±0.060 m, 검은 사각형 끝은 ±0.068 m 라
**영상 세로 방향의 바깥 두 줄 태그는 화면 밖**(가로는 반 시야 0.094 m 라 들어옴). 눈 2~6 은 모두 모서리 칸을 쓰므로 실물에서는 대부분 ID 를 못 읽고 누락으로 기록된다.
태그 라벨(±0.070) + 조준 오차 여유 0.010 을 넣으면 태그까지 0.193 m, 카메라 → 상자 윗면 **0.25 m 이상** 필요(box_look.distances 후보 0.26 은 이미 목록 끝에 있음).
그 거리에서 태그 크기는 원본 약 73 px(찾기용 640 폭으로 줄이면 약 36 px). 관절 한계 안에 드는지는 확인 안 함. `manipulator.yaml` 은 developlcy-oss 담당(할 일 5번 #154 와 같은 문제).
