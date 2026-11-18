@kanichong 무거운 구성(인지 모드 7 · Nav2 켬)도 다시 쟀습니다 (2026-11-15 22:48 ~ 23:45, 실물 Jetson, 설치본 1b9266c).

- 위치 추정 CPU 측정(p7loc, 판마다 bringup 을 새로 띄움) 8 판: `use_teleop:=false use_localization:=true use_nav2:=true use_planner:=false` + 인지 모드 7, 덧씌우기 pws4(인지 · 위치 추정만).
- **8 판 모두 첫 bringup 에서 컨트롤러 4/4** (11-14 오전 같은 구성 30 번 중 25 번 실패, 첫 bringup 9 번 중 6 번 0 ~ 3/4).
- 앞서 말씀드린 '같은 부하로 10 번' 중 8 번입니다. 그 사이 bt_navigator 활성화 실패가 3 판 있었으나 (Nav2 쪽, #276) 컨트롤러와는 무관했습니다.
- 기록: Jetson `~/real_drive/p7a_out/1115_2248` ~ `1115_2338`.
