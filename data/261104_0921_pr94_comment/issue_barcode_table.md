## 무엇
- 바코드 → 정렬 방향 대응을 코드 상수(allocation.py BARCODES)가 아니라 경기 때 받는 표(JSON)에서 읽도록 바꿔 주세요.
- 튜터 안내: 표는 아래 형식이고, 바코드마다 ASC · DESC 는 그때그때 다르게(무작위로) 정해집니다. 지금 코드의 P2-01 · P2-03 = asc, P2-02 · P2-04 = desc 는 예시 한 가지일 뿐입니다.

```json
{"barcodes": [
  {"barcode": "P2-01", "sortOrder": "ASC", "sortLabel": "오름차순"},
  {"barcode": "P2-02", "sortOrder": "DESC", "sortLabel": "내림차순"},
  {"barcode": "P2-03", "sortOrder": "ASC", "sortLabel": "오름차순"},
  {"barcode": "P2-04", "sortOrder": "DESC", "sortLabel": "내림차순"}
]}
```

## 제안
- planner_node 에 표 경로 파라미터(예: barcode_table)를 두고, plan 요청 때 읽기(pucks_file 처럼). sortOrder 는 ASC · DESC 만 받고, 표에 없는 바코드 · 다른 값 · 같은 바코드가 두 번이면 MissionError(오류 기록 · 안전 정지).
- 계획 결과(signal.json 등)에 쓴 표 경로와 고른 sortOrder 를 남기기.
- planner.py 203행의 P2-02 · P2-04 고정 판단도 같은 표를 쓰게.

## 인지 쪽
- /perception/barcode text 는 라벨에 찍힌 CODE128 문자열을 형식 검사 없이 그대로 냅니다(예: P2-01). 인지 쪽 변경은 없습니다.
