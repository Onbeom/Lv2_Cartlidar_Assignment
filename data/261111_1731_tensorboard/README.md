# YOLO 학습 곡선을 TensorBoard 로 보기 · 2026-11-11

E3 등은 학습할 때 TensorBoard 기록을 켜지 않았다. Ultralytics 가 epoch 마다 남긴 `results.csv` 를 TensorBoard 기록으로 바꿔서 본다(다시 학습하지 않음).
- `~/yolo-venv` 에 tensorboard 2.21.0 을 설치했다(11-11).
- `logs/` 에 지금까지의 학습 8개를 모두 바꿔 두었다(11-11 사용자 요청으로 처음 3개에서 늘림).

| 이름 | 시작 가중치 | epoch | 검증 장 수 |
|---|---|---|---|
| sim_v1 | yolo11n.pt | 80 | 400 |
| ft1 · ft1b | sim_v1 best | 18 · 80 | 9 |
| C | ft1b_legacy6 | 80 | 716 |
| D | C last | 80 | 876 |
| E1 | ft1b_legacy6 | 25 | 876 |
| E2 · E3 | ft1b_legacy6 | 80 · 80 | 876 |

검증 세트가 같은 D · E1 · E2 · E3 끼리만 수치를 바로 비교할 수 있다. 나머지는 검증 세트가 달라 곡선 모양만 본다.
- 처음 3개를 띄워서 HTTP 200 과 E3 재현율 80 개 값이 나오는 것을 확인했다.
- 값은 검증 세트(E3 876장) 기준이다. 실물 판정 점수가 아니다.
- 바꾸는 코드: `csv_to_tb.py`. 다른 학습도 `이름=경로` 를 더하면 된다. 다른 results.csv: `261026_1414/runs/{sim_v1,ft1,ft1b}`, `261102_1141_tutor_yolo/runs/C_tutor`, `261103_1205_yolo_E1/runs/E1_aug`.

실행(PC):
```bash
~/yolo-venv/bin/tensorboard --logdir ~/git/lv2_next_module/261111_1731_tensorboard/logs --port 6006
```
브라우저에서 http://localhost:6006 → SCALARS. `metrics/recall(B)` · `metrics/precision(B)` · `metrics/mAP50(B)` 가 정답률 쪽, `train/*_loss` · `val/*_loss` 가 손실이다. 끝낼 때는 그 터미널에서 Ctrl+C.

앞으로 새로 학습할 때 실시간으로 보려면, 학습 전에 Ultralytics 설정을 켠다(확인 안 함, 다음 학습 때 확인):
```bash
~/yolo-venv/bin/yolo settings tensorboard=True
```
그러면 `runs/<이름>/` 에 기록이 생기고, 학습 중에 `--logdir runs` 로 띄우면 epoch 마다 그래프가 늘어난다.
