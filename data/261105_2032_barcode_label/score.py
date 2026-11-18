#!/usr/bin/env python3
"""proto_label.py · barcode_compare --read 출력 세기 (PC). python3 score.py <jsonl>
이름 앞 5자가 P2-0x 면 바코드 장(정답 = 그 문자열), 아니면 바코드 없는 장(무엇이든 읽으면 잘못 읽기)."""
import json
import re
import sys
from collections import Counter

ok, n, wrong, neg, negread, ms = Counter(), Counter(), 0, 0, 0, []
for line in open(sys.argv[1]):
    if not line.strip():
        continue
    r = json.loads(line)
    lab = r['name'][:5]
    ms.append(r.get('ms', 0))
    if re.fullmatch(r'P2-0[1-4]', lab):
        n[lab] += 1
        if r['ok'] and r['text'] == lab:
            ok[lab] += 1
        elif r['ok']:
            wrong += 1
    else:
        neg += 1
        negread += bool(r['ok'])
ms.sort()
print(' '.join(f'{k} {ok[k]}/{n[k]}' for k in sorted(n)), f'| 합 {sum(ok.values())}/{sum(n.values())} | 틀림 {wrong} | 바코드 없는 장 읽음 {negread}/{neg}',
      f'| ms 중앙 {ms[len(ms) // 2]:.0f} · 최대 {ms[-1]:.0f}' if ms else '')
