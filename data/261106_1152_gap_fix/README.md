# loc_monitor 거짓 LOST(gap) 고침 시험 (Gazebo · Isaac 모의, 실물 아님)

고침: 브랜치 `fix/loc-monitor-gap-stationary`(`gapfix.diff`, 기준 커밋 `base_commit.txt`). amcl 일 때 map → odom TF 시각이 계속 바뀌는 동안은 /amcl_pose 가 끊겨도 LOST 로 보지 않는다. TF 시각이 gap_s 넘게 안 바뀌면 LOST(AMCL · 스캔 죽음).

## 1 차 설계(버림): `results_v1/`
odom 이 그대로면 LOST 아님. F1 · F2 LOST:gap 0 이었지만 **F3 불통과**: AMCL 을 죽이자 TF 가 낡아 Nav2 가 차를 세웠고, 차가 멈추니 odom 이 그대로라 LOST 가 끝까지 안 남. 도착 4/12.
bag 에서 AMCL 은 pose 끊김 동안에도 map → odom 을 0.1 s 마다 냈고(T2b 끊김 44 번, F1 12 번, TF 간격 최대 0.100 s), 죽으면 멈춤(F3 stamp 53.0) → 2 차 설계.

## 판정 기준(직접 정함)
- F1 · F2: LOST:gap 0. F3: AMCL 을 죽이면 LOST:gap(멈춰도). I1 · I2: 이전 K0 · K1 그대로.
- F0 · F4 는 판정이 아니라 원인 가르기: 도착 수가 줄어든 것이 gap 고침 때문인지, 남은 LOST:residual 때문인지.

## 결과 표
| 시험 | 값의 의미 · 단위 | 예상 | 결과 |
|---|---|---|---|
| F1 · F2 Gazebo 주행 12 곳, loc_map 일치 | LOST 이유별, /initialpose 횟수, 도착 수, 잔차 | LOST:gap 0. 1 차와 같이 LOST:residual 은 남아 도착 수는 낮을 수 있음 | **통과**. 두 판 모두 LOST:gap 0(고치기 전 40), gap_alive 13 · 15 번이 LOST 대신 넘어감. /initialpose 6 · 6(고치기 전 47), LOST:residual 6 · 6, 도착 9/12 · 9/12(고치기 전 9/12) |
| F3 주행 중 AMCL 강제 종료 | 종료 뒤 첫 LOST 이유 · 시간(s, 모의) | LOST:gap, 약 0.5~1 s(TF 시각이 앞당겨져 있어도 바뀜 여부로 잼) | **통과**. 종료 직전 시계 53.30 s, LOST:gap 54.17 s(약 0.87 s). 1 차 설계는 끝까지 안 남 |
| F0 loc_monitor 끔 | 도착 수 | 팀 README 와 같은 12/12. 이 도구가 맞는지 기준 | **예상과 다름: 8/12**(B3:S~B4:N 넷 error 104). 팀 README 12/12 를 이 도구로 재현 못 함, 다른 점은 안 가림 |
| F4 잔차 문턱 0.10(사실상 잔차 LOST 끔) | 도착 수, LOST 이유별 | 12/12 가까이. gap 판단이 주행을 방해하지 않음을 보임. 못 미치면 다른 원인 | **9/12**, LOST 0 · /initialpose 0, gap_alive 14. 그래도 error 104 로 중단(B3:S · B3:N · B4:S) → 중단은 loc_monitor 때문이 아님 |
| I1 Isaac K0 amcl | G1~G5, SUSPECT · LOST, gap_alive 수 | B4 와 같음, gap_alive 0 | **통과**. G1~G5, RMSE 0.0150 · 최대 0.0293, SUSPECT 0 · LOST 0, gap_alive 0, 잔차 최대 0.0343, G3 0.0186 |
| I2 Isaac K1 amcl | R1~R5, 감지(s), 다시 찾은 오차(m) | 11-03 과 같음(1 차 설계 때 0.67 s · 0.0083 m) | **통과**. R1~R5, G1 7/7, G3 0.0088, 감지 0.63 s, 다시 찾은 오차 0.0193 m · 0.32°, 감지 → OK 3.1 s, gap_alive 0 |

## 판정: **통과(모의)**
F1 · F2 · F3 · I1 · I2 모두 기준 통과. Isaac 에서 gap_alive 가 0 이라 EKF 구성의 동작은 그대로다.

## 한계
- Gazebo 도착 수는 고치기 전과 같다(9/12). error 104(Controller patience exceeded)는 loc_monitor 를 꺼도 난다(F0 8/12, F4 9/12). 원인은 안 봤다(Nav2 · Gazebo 쪽, 주행 담당 영역). 고침의 효과는 도착 수가 아니라 거짓 LOST · 재설정이 사라진 것이다.
- Gazebo 의 LOST:residual 6 번이 거짓인지 모른다. 참값이 없고, 이 구성은 EKF · IMU 없이 스키드 odom 을 그대로 쓴다. 실물 문턱은 C8.
- map → odom 시각 판단은 AMCL 이 tf_broadcast true 로 스캔마다 TF 를 낸다는 것(bag 으로 확인, 코드는 안 읽음)에 기댄다. tf_broadcast 를 끄면 pose 끊김이 곧 LOST 로 돌아가 예전과 같다.
- 실물 EKF 에서는 안 재 봤다.
