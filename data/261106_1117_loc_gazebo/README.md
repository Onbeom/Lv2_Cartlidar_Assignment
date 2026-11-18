# loc_monitor 위험 두 가지 확인 (Gazebo 모의, 실물 아님)

확인할 것: ① `map:=` 만 바꾸고 `loc_map:=` 을 안 바꿀 때 거짓 LOST 가 나는가 ② navigation README 의 'loc_monitor 시작 직후 LOST(gap) → 전역 탐색이 엉뚱한 자세'가 재현되고 원인이 무엇인가.
구성: dev 30c2d1f 고정 사본(`team_src/`), 발제 SDF 세계, `/map` = map_amcl · `/map_objects` = map_nav, 출발 (0.69365, 0.95, π), 팀 navigation 패키지 그대로, 창 없음. EKF 없음(Gazebo /odom 이 바로 odom → base_link).
어느 쪽도 실물 동작을 증명하지 않는다. 특히 Gazebo 는 EKF · 센서 잡음 조건이 실물과 다르다.

## 판정 기준(직접 정함)
- 위험 ①: T2a(불일치)에서 거짓 SUSPECT · LOST 가 나고 T2b(일치)에서 줄어들면 위험 확인.
- 위험 ②: 멈춘 채 시작한 T1 에서 LOST(gap) 이 나면 재현. 원인 가설은 정지 중 /amcl_pose 끊김 시각과 /odom 변화로 맞춰 본다.

## 결과 표
| 시험 | 값의 의미 · 단위 | 예상 | 결과 |
|---|---|---|---|
| T1a 정지 · loc_map 일치 | Gazebo 60 s 켠 뒤 Nav2 · loc_monitor 시작, 멈춘 채 관찰. LOST 이유 · 횟수, /amcl_pose 최대 간격(s), 다시 넣은 자세 오차(m) | gap LOST 재현. Gazebo /odom 은 정지 때 변화가 정확히 0 이라 AMCL 이 갱신 · 발행을 안 함. 다시 넣은 자세는 지도가 맞아 오차 작음 | **예상과 다름: LOST 0**. /amcl_pose 최대 간격 0.100 s, 잔차 p50 0.000 · 최대 0.0012 m. /odom 변화가 3.45e-18 m 로 정확히 0 이 아니었음 |
| T1b 정지 · loc_map 기본(slam_walls) | 같음 | gap LOST 재현. 전역 탐색이 다른 세계 지도로 찾아 오차가 커지거나 못 찾음 | **예상과 다름: LOST 0**. 간격 0.100 s. 잔차 p50 0.0258 · 최대 0.0308 m(같은 자리 T1a 의 0.000 과 달리 지도 차이가 보임, 문턱 0.034 아래) |
| T2b 주행 12 곳 · loc_map 일치 | 도착 수, SUSPECT · LOST 횟수와 이유, 잔차 최대(m) | 12 중 대부분 도착. 잔차 문턱 0.034 는 Isaac 기준이라 칸 0.025 지도에서 SUSPECT 는 조금 날 수 있음. 목표 사이 정지에서 gap LOST 가능 | 9/12 도착(L2 · L3 · HOME 중단). LOST:gap 40 · LOST:residual 8 · SUSPECT 30, /initialpose 47 번. 잔차 p50 중앙 0.0213 · 최대 0.1(잰 값의 상한). 끝 상태 RECOVERING |
| T2a 주행 12 곳 · loc_map 기본 | 같음 | 동쪽 · 북쪽에서 잔차가 7~10 cm 까지 커져 거짓 SUSPECT · LOST, 전역 탐색이 위치를 뒤흔들어 T2b 보다 나쁨 | 7/12 도착. LOST:gap 7 · LOST:residual 2 · LOST:confirm_timeout 8 · SUSPECT 9, 전역 탐색 14 번, **165 s 에 FAILED**. 잔차 p50 중앙 0.0767 · 최대 0.1 |

## 판정
- **위험 ① 확인(Gazebo 모의).** 같은 정지 자리에서 잔차 p50 0.000 → 0.0258 m, 주행 중 0.0213 → 0.0767 m. T2a 는 CONFIRM 을 통과하지 못해(confirm_timeout 8) FAILED 로 끝났다. FAILED 면 /localization/lock 이 계속 true 라 use_loc_lock 구성에서는 Nav2 가 막힐 수 있다(이 시험에는 twist_mux 가 없어 잠금 자체는 안 봄).
- **위험 ② '시작 직후 LOST' 는 재현 안 됨**(켜 둔 시간 60 s 만 봄, '오래'의 길이는 README 에 없음). 대신 **목표 도착 뒤 정지에서 거짓 LOST(gap) 를 재현**했다.

## 원인(잰 것)
- T2b 끊김 44 번을 시각으로 대조: 앞 14 번 중 13 번은 그 동안 /odom 이동 · 방향이 정확히 0(나머지 1 번 이동 1.66e-11 m · 방향 0.0094 rad). amcl.yaml `update_min_d · update_min_a` 0 이면 변화가 정확히 0 일 때 AMCL 이 갱신 · 발행을 안 하는 것으로 본다(끊김과 일치, AMCL 코드는 안 읽음).
- T1 은 한 번도 안 움직여 /odom 변화가 3.45e-18 m 로 남아 AMCL 이 계속 냄. 움직인 뒤 멈추면 정확히 0 이 됨.
- `loc_monitor.tick` 은 pose 가 0.5 s(gap_s) 끊기면 멈춰 있는지 보지 않고 LOST. LOST → /initialpose → OK → 다시 끊김이 되풀이된다.
- Isaac(EKF 거침) B4 A1 · A2 · B1 은 정지 구간이 있어도 >0.5 s 끊김 0 번. 그래서 Gazebo /odom 을 EKF 없이 바로 쓰는 조건에서 드러난 것으로 본다. 실물 EKF 에서 안 나는지는 재지 않았다.

## 아직 모르는 것
- T2b 끝 3 곳 중단 원인. 거짓 LOST 되풀이가 섞여 잔차 · 도착 수만으로는 가를 수 없다. 팀 README 는 loc_monitor 끈 Gazebo 12/12.
- 다시 찾기가 방향을 14~20° 옮겨 넣은 일이 있다(T2b 40 · 51 · 61 · 91 · 103 · 175 s). 이 시험엔 참값이 없어 옳았는지 모른다.
- 주행 잔차 p50 0.021 m 는 Isaac 판(약 0.01)보다 크다. 지도 칸 0.025 · Gazebo AMCL 오차 · 되풀이 재설정 중 어느 것 때문인지 안 가렸다.
- AMCL 의 'Failed to transform initial pose' 경고가 T2b 46 번(Isaac K1 도 2 번 중 1 번). 멈춰 있을 때는 영향이 작을 것으로 보지만 재지 않았다.
