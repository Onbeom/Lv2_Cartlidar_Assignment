# 시연 전 코드 점검 (11-16 08:5x, dev 8cad7df, 읽기만)

확인 표시: [확인] = 코드를 직접 봄, [추정] = 점검 보고만 있고 실물·계산 확인 안 함.

## 시연을 멈출 수 있는 것
1. [확인] loc_monitor(C++, #310) 가 FAILED 가 되면 빠져나오지 못함. lost() 가 FAILED 면 바로 return, 다시 넣기 경로 없음 → /localization/lock true 유지 → Nav2 잠김. 현장 대응: bringup 다시 띄우기. (내 담당 — 시연 뒤 고침)
2. [확인] /scan_merged 는 앞 라이다 스캔이 올 때만 발행 (loc_scan_merge_node.cpp 6 · 90~157). 앞 라이다가 끊기면 AMCL · costmap 이 멈춤. 시작 전 `ros2 topic hz /scan_front` 확인.
3. [확인 일부] 박스 앞 후진(drive_node 후진 단계)은 뒤 카메라 stop_m 0.40 m 안에 점이 있으면 멈춰 기다리고 시간 초과면 navigate 실패. [추정] map_nav 로 잰 몇몇 상자 자리(B1:S · B3:N · B4:N · B5:N) 뒤 빈 거리 0.32~0.33 m < 0.40. 현장: 로그 '후진 대기: 뒤 막힘' 이 나오면 이것.
4. [추정] TFmini 비상 정지(stop 0.25 · 해제 0.35 × 3) 가 방향과 무관하게 Nav2 출력 전체를 막음 — 좁은 자리 회전에서 걸릴 수 있음.
5. [추정] 잔차 문턱 loc_res_thr 0.034 m · AMCL sigma_hit 0.03 은 모의 값 → 실물에서 SUSPECT/LOST 잦을 수 있음. 명령줄 `loc_res_thr:=0.05` 로 넓힐 수 있음(추정).
6. [추정] AMCL 초기 자세 공분산을 넣는 곳 없음 → 출발 자리 (0.694, 0.950, 180°) 에 정확히 놓거나 RViz 2D Pose Estimate.

## 참고
- 지도 기본값 = map_amcl · map_nav (SDF), sim 값 남은 것 · TF 이중 발행 없음 (두 점검 모두).
- icr_x −0.0658 은 Isaac 값(#49 실물 미측정), Nav2 Spin 복구 · drive 의 5 cm 안 spin 은 4 륜 스키드에서 회전 중심 차이(검사 0.26 m · 실제 약 0.32 m, 추정).
- YOLO 엔진 경로 고정 — 없으면 WARN 한 줄 뒤 색 검출기. 로그 'detector.engine_path 를 쓰지 못해' 확인.
- TAG_VIEW 는 TODO 값, 측량 다시 보기 자세로 씀.
- DRIVE 자세: 앞 위 D435 화면 가운데에 집게 · 팔 카메라가 들어옴 (팀 URDF FK, 좌우 ±5°, 화면 중심 −6° ~ +20° 아래, 링크 점만 계산). Nav2 는 라이다만 써서 주행에는 영향 없음, 정면 검출에는 영향 가능.

## 신호등 · 바코드 → 미션 · 계획기 길 (11-16 추가, dev 8cad7df 읽기만)
길: perception_node → /perception/signal · /perception/barcode (Reliable depth 1, 켜져 있는 동안 계속 발행) → mission_node(observe: 둘 다 왔는지만 봄) · mission_planner_node(문자열 해석) 가 각자 구독. 미션 단계 = observe → 측량(survey) → plan.
- [확인] 길 자체에 끊긴 곳 없음: 토픽 이름 · QoS · 메시지 형 일치, CONFIRMED 뒤 순서는 안 바뀜(signal_sequence.hpp 290), 바코드 못 읽은 장은 미션 · 계획기가 무시.
7. [확인] 바코드 문자열 검사는 plan 단계에만 있음(P2-01~04 와 글자까지 같아야 함). 틀리게 읽혀도 observe 는 통과 → 측량을 다 한 뒤 plan 에서 '미등록 바코드' FAILED, 재시도 없음.
8. [확인] 계획기는 미션 observe 와 따로 '마지막 CONFIRMED · 마지막 READ' 를 기억(지우는 곳 없음). bringup 을 안 끄고 미션을 다시 시작해 observe 가 실패하면 앞 판 값이 남아 있음 → 다시 할 때는 bringup 을 새로 띄움.
9. [확인] 신호 CONFIRMED 는 한 번 확정되면 READ_SIGNAL 을 끌 때까지 안 바뀜. confirm_clear_cycles 1 이라 높이로 분명한 묶음 하나로 확정 → 그 한 묶음이 틀리면 그대로 감(다시 세기 없음).
10. [추정] 신호등 (0.08, 0.95) 은 출발 자세 정면 0.61 m · 낮은 곳. DRIVE 자세면 집게가 가리는 범위와 겹침. observe 는 barcode_look(팔이 왼쪽 위 라벨로 감) 뒤에 세므로 비킬 것으로 보이나 계산 · 실물 확인 안 함. 가리면 '모름' 으로 처리돼 묶음이 안 세어짐 → 180 s 시간 초과.
11. [추정] 신호 HSV 문턱 주석에 '가정값 · 모의' 가 남아 있음(height_color 만 11-10 실물 반영). barcode_look 역기구학 자세가 실물 팔에서 나오는지, 팔 카메라 0.3 m 에서 실물 라벨이 읽히는지는 이 세션에서 확인 안 함.
12. [계산 · 모의] DRIVE 자세면 앞 위 D435 화면의 26.7 % 를 팔이 가림(가로 133~507 px, 세로 98~479 px, 대부분 gripper_roll_link 24.2 %). 출발 자세에서 신호등 세 칸은 화면 (320, 151~200) px = 모두 집게 뒤(가림 100 %). DRIVE 로는 신호등을 못 셈(후보 없음 → SEARCHING → 180 s 시간 초과). 미션은 barcode_look 자세에서 세므로 그 자세에서 비키는지가 관건 — 그 자세 관절각은 실행 때 IK 로 정해져 계산 못 함, 실물 확인 필요. 그림 photo/261116_drive_front_cam_occlusion.png, 계산 drive_occl.py (집게 벌림은 0 으로 둠, 카메라 내부값 가정).
13. [계산 · 모의] STOW 자세(모델 관절값 그대로)면 앞 위 D435 가림 1.3 %(왼쪽 아래 구석), 신호등 가림 0 %. 다만 STOW 는 이름 자세로 못 보냄(pose.blocked, 켜질 때 정리의 역순으로만), 집기 · 팔레트는 DRIVE · ALIGN · TRANSIT_TILT 에서만 시작, 실물 J1 기준이 11-14~16 사이 여러 번 바뀜 → 시연 전 주행 자세를 STOW 로 바꾸지 않음(팔 담당 파트). 주행(Nav2)은 라이다 · TFmini 만 써서 DRIVE 가림과 무관. 앞 D435 는 observe(신호) · 측량 · verify · 상자 재기에서만 쓰고 그때는 각 단계의 팔 자세임.
