## 무엇을 봤나 (실물 Jetson, 11-13 ~ 11-14)
`bringup.launch.py use_teleop:=false use_localization:=true use_nav2:=true` 를 정지 상태로 여러 번 띄웠을 때, 60 s 뒤 활성 컨트롤러 수(`Configured and activated` 줄):
- 11-14 02:31 ~ 03:12 첫 bringup 9 번 중 **6 번 실패**(0/4 · 1/4 · 3/4), 다시 띄워도 3 번 더 실패. 11-13 에도 5 번 중 2 번.
- 실패한 판도 OpenCR 은 정상: `Successful 'activate' of hardware 'OpenCRSystem'`. 실패는 spawner 쪽:
  - `Could not contact service /controller_manager/load_controller`
  - `Failed to acquire lock in 20 seconds. Attempt N of 5 failed`
- 실패하면 `/joint_states` 가 없거나 diff_drive · imu 가 없어 odom TF 가 없음 → 인지 팔 처리가 'TF 대기' 로 멈춤.

## 추정
bringup 직후 노드 수십 개(카메라 3 · 인지 · Nav2 · 라이다 · EKF …)가 한꺼번에 떠서 CPU 가 몰릴 때 controller_manager 서비스 응답이 늦음. spawner 4 개가 같은 잠금을 기다리며 시간을 넘김.

## 부탁 (하나 또는 여럿)
- spawner 의 `--controller-manager-timeout` · `--service-call-timeout` 늘리기, 또는 컨트롤러 4 개를 spawner 하나로 띄우기(잠금 다툼 없앰).
- 컨트롤러를 먼저 띄우고 무거운 노드(인지 · Nav2)는 `/joint_states` 나온 뒤 띄우기(TimerAction · event handler).

## 한계
- 정지 상태 · 미로 밖. 기록: Jetson `~/real_drive/p7a_out/1114_0231 ~ 0312` 의 `bringup_fail_*.log` · `summary.txt`.
