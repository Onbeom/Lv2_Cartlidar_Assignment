@kanichong 실물 Jetson 확인 결과입니다 (2026-11-15 16:03 ~ 19:40).

## 조건
- 16:03 3 판: 설치본 c453996 (`fix/stow-j1-real-20261115`: dev 1767e0f + STOW J1 −1.287). bd1b4a0 · 6577f89 · b020f52 · f0759ad 모두 들어 있음.
- 11-14 실패 때와 같은 인자: `use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false`
- bringup 을 3 번 띄우고 판마다 90 s 뒤 측정, 끈 뒤 20 s 쉬고 다음 판. 다른 bringup · 남은 노드 없음 확인 뒤 시작. 팔은 1 판 STOW, 2 · 3 판은 앞 판에서 켠 토크로 HOLD.

## 결과
| 판 | 컨트롤러 (로그 · list_controllers) | spawner 실패 줄 | /rc100/raw | /joy | /tf |
|---|---|---|---|---|---|
| 1 | 4/4 · 4 | 0 | 17.8 Hz | 20.0 Hz | 71 Hz |
| 2 | 4/4 · 4 | 0 | 17.8 Hz | 19.8 Hz | 76 Hz |
| 3 | 4/4 · 4 | 0 | 18.0 Hz | 20.0 Hz | 78 Hz |

- 컨트롤러 4 개 모두 하드웨어 활성 뒤 약 1.5 s 안에 켜짐 (`Failed to acquire lock` · `Could not contact service` 0 줄). 같은 인자인 11-14 13:03 미로 주행 bringup 에서 바퀴 spawner 가 `Could not contact service` 를 되풀이하며 잠금을 쥐고 멈췄던 것은 이번 4 판에서 나오지 않았습니다.
- `/joy` 3 판 모두 20 Hz (11-14 에는 bringup 이 띄운 rc100_joy 가 `/joy` 를 한 번도 안 냄).
- `/tf` 녹화: 같은 인자의 bringup (19:28 판) 을 띄운 채 `ros2 bag record --topics /tf /tf_static` 13 s → `/tf` 1043 개 (약 78 Hz) · `/tf_static` 3 개. 11-14 의 '/tf 미녹화' 는 이 설치본에서는 재현되지 않았습니다 (단, 녹화 방법이 다름: 11-14 는 `-e` 정규식 · bringup 14 분 뒤 시작, 이번은 `--topics` · 띄운 채 13 s). (c275 스크립트 안의 녹화는 터미널에서 rosbag2 키보드 처리 때문에 끝나지 않아 따로 녹화함 — 시험 스크립트 문제)
- 19:36 에 한 판 더 (설치본 fd6bcbf, 팔 HOLD): 컨트롤러 4/4 · spawner 실패 0 · /rc100/raw 17.8 Hz · /joy 20.0 Hz · /tf 74 Hz
- 그 밖: rc100_joy 가 끌 때 (SIGINT 뒤) `ExternalShutdownException` 으로 exit code 1 — 실행 중에는 문제 없음. 실행 중 ERROR 는 RealSense 설정 파일 없음 · EKF 주기 놓침 1 번뿐.

## 한계
- 4 판 · 정지 상태 · 인지 · Nav2 를 끈 미로 주행 인자뿐입니다. 인지 모드 7 · Nav2 를 켠 무거운 구성 (11-14 오전 30 번 중 25 번 실패) 은 아직 다시 안 쟀습니다. 앞서 제안한 '같은 부하로 10 번' 확인은 그대로 남아 있습니다.
- 처음 시도한 두 번은 판정에서 뺐습니다: 첫 판이 남긴 노드 (robot_state_publisher, 옛 `/robot_description`) 를 새 ros2_control_node 가 받아 옛 대기 범위로 검사해 시작이 거부됨 (bringup 문제 아님, 시험 정리 잘못).
