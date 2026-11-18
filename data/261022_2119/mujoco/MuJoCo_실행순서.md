# MuJoCo 로딩 검사 실행 순서 (261022_2119, 2026-10-23)

기준 교안: velog "2026-10-22 URDF,USD -> Mujoco"(https://velog.io/@roboticsmaster/2026-10-22-URDFUSD-Mujoco) 3·9·11절. 교안은 Isaac Sim 6.2 기준이지만 MuJoCo 검사는 Isaac Sim과 상관없는 일반 Python venv에서 한다(교안 3절).

이 검사는 **모델 로딩·수치 안정 검사(모의)**다. 중력 0, 입력 없음, 500 스텝이며 주행·중력·접촉·제어 시험이 아니다(교안 9절).

## 준비된 파일 (이 폴더)

| 파일 | 내용 |
|---|---|
| `waffle_4wd_mujoco.urdf` | 261022_2119 URDF 사본. 바꾼 것 두 가지: ① `package://…/` 경로를 상대 경로 `../urdf/waffle_4wd_description_261022_2119/`로(교안 5절: MuJoCo가 `package://`를 푼다고 가정하지 않는다) ② `<mujoco><compiler strippath="false" discardvisual="true"/></mujoco>` 추가(표시 메시가 DAE라 MuJoCo가 읽지 못함, 충돌 메시 STL만 씀). 원본 URDF는 그대로 |
| `mujoco_smoke.py` | 교안 9절 코드 그대로 |

## 예상 결과 (가설, 실행 전)

| 항목 | 예상 | 이유 |
|---|---|---|
| status | `PASS_LOAD_STEP_ONLY` | 로딩·500 스텝 동안 값이 유한하면 나온다 |
| nq / nv | 11 / 11 | 움직이는 관절 11개(바퀴 4, 팔 5, 집게 2). 루트(`base_footprint`)는 MuJoCo의 URDF 읽기에서 월드에 고정되어 자유 관절이 없다 |
| nu | 0 | URDF에는 구동기(actuator)가 없다 |
| 경고 | `Geom with duplicate name 'convex_0_30'`, `'convex_0_31'` 2줄 | URDF 충돌 이름이 겹치는 곳이 있어 MuJoCo가 이름 없이 만든다. 오류 아님 |

(실행 후 정정 2026-10-23: 경고는 2줄이 아니라 같은 종류 43줄이었다. 준비 확인 때 출력 끝부분만 보고 2줄로 잘못 적었다. 43은 URDF에서 겹치는 충돌 이름의 중복 횟수 합(`convex_0_0~31` 32, `part_0` 4, `tire_cylinder` 3, `part_5` 2, `part_6` 1, `lidar_carrier_envelope` 1)과 같다.)

(참고: 이 세션이 Isaac Sim venv에 들어 있던 MuJoCo 3.11.0으로 준비 확인을 한 번 돌려 위와 같은 값이 나왔다. 교안 버전 3.16.0으로 사용자가 돌린 결과를 공식 결과로 쓴다.)

## 순서 (모두 PC)

1. 일반 Python venv 만들기 (필수, 교안 3절: Isaac Python과 섞지 않기 위해)
```bash
python3 -m venv ~/git/lv2_next_module/261022_2119/mujoco/.venv
```

2. 교안과 같은 버전 설치 (필수, 인터넷에서 받음)
```bash
~/git/lv2_next_module/261022_2119/mujoco/.venv/bin/python -m pip install numpy mujoco==3.16.0 usd-core==26.8
```

3. 설치 확인 (필수, 경로와 버전 3.16.0이 나와야 함)
```bash
~/git/lv2_next_module/261022_2119/mujoco/.venv/bin/python -c "import sys,mujoco; print(sys.executable); print(mujoco.__version__)"
```

4. 로딩·500 스텝 검사 (필수, 결과 폴더가 이미 있으면 멈추므로 다시 할 때는 run2처럼 이름을 바꾼다)
```bash
cd ~/git/lv2_next_module/261022_2119/mujoco && .venv/bin/python mujoco_smoke.py waffle_4wd_mujoco.urdf run1
```

5. 결과 보기 (필수)
```bash
cat ~/git/lv2_next_module/261022_2119/mujoco/run1/report.json
```

6. 뷰어로 열기 (선택 · 권장, 화면 확인은 사용자가 한다. 창을 닫으면 끝)
```bash
~/git/lv2_next_module/261022_2119/mujoco/.venv/bin/python -m mujoco.viewer --mjcf=$HOME/git/lv2_next_module/261022_2119/mujoco/waffle_4wd_mujoco.urdf
```
   - 볼 것: 로봇 충돌 형상(회색 덩어리들)이 제자리에 모여 있는지, 팔·바퀴가 떨어져 나간 부품이 없는지. 표시 메시는 버렸으므로 외형이 Isaac Sim과 다르게 보이는 것은 정상이다.
   - 뷰어는 CSV를 재생하는 것이 아니다(교안 9절).
   - (2026-10-23 수정) 처음에는 `run1/compiled.xml`을 열게 했으나, 그 파일의 메시 경로가 `../urdf/…`로 저장되어 `run1/` 기준으로는 파일을 찾지 못한다(`Error opening file`). 그래서 같은 입력인 `waffle_4wd_mujoco.urdf`를 연다.
   - 뷰어는 URDF 기본 중력을 켠 채 시뮬레이션을 돌린다(검사 스크립트와 달리 중력 0이 아님). 구동기가 없어 팔이 아래로 처지거나 흔들리는 것은 정상이다. 형상만 볼 때는 창이 뜨자마자 `Space`로 멈춘다.

7. 결과 알려 주기: 5번의 `report.json` 내용과 6번 화면 확인 결과. 이 세션이 교안 11절에 맞춰 `실행기록.md`(입력 URDF, 버전, 명령, 결과, 검사하지 않은 범위)를 작성한다.

## 하지 않는 것

- USD를 MuJoCo에 바로 넣는 경로(교안 1절: 기본 실습 아님)
- 중력·바닥·구동기를 넣은 주행 시험(교안 9절 끝: 별도 MJCF·장면 코드 필요)
