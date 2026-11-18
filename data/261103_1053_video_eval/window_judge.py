"""창 판정(사용자 11-03: 영상이니 여러 장으로 추측) — 뽑은 장의 셈 안 정답마다, 원 영상(6 fps)에서 그 장 앞뒤 ±W 프레임 안에
문턱 0.6 이상 · 규칙 맞는(앞 = 1 m 안 클래스 / 1~3 m 색, 뒤 = 있다/없다) 검출이 그 정답 상자와 IoU ≥ 0.3 로 몇 장 있는지.
'1장 이상'(창 안에 한 번은 봄) · '2장 이상'(등록부 독립 관측 2개에 가까움) 비율. IoU 0.3 은 장 사이 카메라 움직임 여유(움직임이 크면 낮게 나옴 = 보수적).
쌓인 퍽(stacked.json)은 뺌. 사용: window_judge.py [W...]"""
import json, os, sys, contextlib, io
from pathlib import Path
HERE = Path(__file__).resolve().parent
os.environ.update(JUDGE_CAMS='judge_front,judge_rear,video_front,video_rear', JUDGE_STACKED='stacked.json')
src = open(HERE / 'judge.py').read(); src = src[:src.index("side = lambda b")]
ns = {'__file__': str(HERE / 'judge.py')}; sys_argv = sys.argv; sys.argv = ['x']
with contextlib.redirect_stdout(io.StringIO()): exec(src, ns)
GT, META, iou = ns['GT'], ns['META'], ns['iou']
WIN = json.load(open(HERE / 'win_D.json'))
SRC = {}
for cam in ('judge_front', 'judge_rear', 'video_front', 'video_rear'):
    for e in json.load(open(HERE / cam / 'auto_label.json')):
        p = Path(e['source']); SRC[f'{cam}/{e["image"]}'] = (p.parent.parent.name, int(p.stem))
AGN = {'judge_rear', 'video_rear'}
for W in [int(x) for x in sys_argv[1:]] or [0, 3, 6]:
    print(f'== 창 ±{W} 프레임(±{W / 6:.1f} 초)')
    for cam in ('judge_front', 'judge_rear', 'video_front', 'video_rear'):
        n = a1 = a2 = 0
        for key, g in GT.items():
            if not key.startswith(cam + '/'): continue
            vid, i0 = SRC[key]
            for x in g:
                if x.get('stk') or x['z'] is None or x['z'] > 3.0: continue
                near = x['z'] <= 1.0; n += 1; hits = 0
                for j in range(i0 - W, i0 + W + 1):
                    for c, s, *b in WIN.get(f'{vid}/{j}', []):
                        if s < 0.6 or iou(b, x['b']) < 0.3: continue
                        if cam in AGN or (c in x['cs'] if near else c % 2 in {k % 2 for k in x['cs']}):
                            hits += 1; break
                a1 += hits >= 1; a2 += hits >= 2
        print(f'  {cam}: 정답 {n} | 창 안 1장 이상 {a1 / n:.3f} | 2장 이상 {a2 / n:.3f}')
