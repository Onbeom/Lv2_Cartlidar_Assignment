**#192 평가 중 확인한 것: 세운 직육면체 n5 집기 → 칸 넣기의 '칸에 못 앉음' (Isaac 모의, 2026-11-11)**

#128 에서 정리된 '손 안 퍽 위치 가정 (`--est`) 오차' 와 같은 원인임을 512 대로 다시 확인했습니다. 새로 고칠 것을 요청하는 글은 아니고, 기록으로 남깁니다.

| 시험 (512 대, mission 배치 (seed 고정 안 함 = 실행마다 무작위 표본), 옆 퍽 없음) | 퍽을 세워 둔 자리 | 칸 넣기 IK 의 손 안 퍽 위치 | 칸에 못 앉음 / 다시 집기를 시작한 대 |
|---|---|---|---|
| R0b' | 각본이 누운 퍽을 세워 놓은 자리 | 가정 (`--est`) | 59 / 440 (13.4 %) |
| R0s | 같은 자리 D 에 퍽을 바로 세워 둠 | 가정 (`--est`) | 81 / 487 (16.6 %) |
| R0t | 미션의 일반 세운 퍽 자리 | 가정 (`--est`) | 59 / 270 (21.9 %) |
| R0s-t | R0s 와 같음 | **참값** (`--est` 없음) | **6 / 488 (1.2 %)** |

- 네 시험 모두 n5 정책으로 다시 집고 같은 `pallet_place_traj.json` 궤적으로 칸에 넣었습니다. 차이는 퍽을 세워 둔 자리와 손 안 퍽 위치 (가정 / 참값) 뿐입니다.
- 못 앉은 대와 성공한 대를 가르는 것은 가정 오차입니다. 가정 수평 오차가 3 mm 를 넘은 대가 못 앉은 대에서 76 ~ 84 %, 성공한 대에서 35 ~ 38 % 입니다. 집기 끝 기울기 · 운반 중 기울기 · 칸 번호로는 갈리지 않습니다.
- 참값으로 놓으면 다른 실패가 조금 보입니다: 놓기 IK 놓침 7 대 (대부분 1 열, 시간 끝), 손가락-몸체 1 N 넘은 대 7 (최대 39.2 N).
- R0t 는 운반 중 퍽-몸체 1 N 넘은 대가 23 대 (R0s 5 대) 로 많았습니다. R0t 는 누운 퍽을 그 자리에 세워 놓는 경로를 찾은 270 대만 셌습니다 (분모 270 · 440 · 487 이 다른 이유).

시험 명령 (팀 dev 055f705 의 `policy/manipulator` 작업본, `chain45_play.py` 에 평가 옵션만 더함. 팀 저장소 코드는 바꾸지 않음):

```bash
./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_spot_virtual 1 --est --seconds 60
```

```bash
./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --est --seconds 60
```

```bash
./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --lie_stand_at_nb 1 --est --seconds 60
```

```bash
./run_rl.sh chain45_eval 512 --lie_regrasp --lie_layout mission --lie_nb_prob 0.0 --lie_stand_start 1 --seconds 60
```

(위에서부터 R0b' · R0s · R0t · R0s-t. `--lie_spot_virtual` · `--lie_stand_start` · `--lie_stand_at_nb` 는 작업본에만 있는 평가 옵션입니다.)

한계: 모의 시험만입니다. 실물에서는 확인하지 않았습니다. 실물 미션도 칸 넣기에서 손 안 퍽 위치를 가정으로 쓰므로 (#128: 팔 카메라로 쥔 퍽을 볼 수 없음), `--est` 의 가정 오차 모델이 실물과 비슷하다면 세운 퍽을 칸에 넣을 때 13 ~ 22 % 가 못 앉을 수 있습니다 (실물 오차 크기는 확인하지 않았습니다).
