"""노드 box_look 은 (거리 목록 × 각도 목록) 순서로 처음 풀리는 자세를 씀 → 목록 하나로 두 번 보기(앞 o1 · 뒤 o2)가 9 칸을 덮는지 (계산).
two_view.py 의 보기 표(views)를 그대로 써서, 후보 목록 순서마다 노드가 고를 자세를 흉내 냄.
  python sim_list.py <dev urdf>"""
import itertools, sys, os, io, contextlib
dev = sys.argv[1]
sys.argv = ["two_view.py", dev, "--face", "0.127"]
ns = {"__name__": "tv", "__file__": os.path.join(os.path.dirname(os.path.abspath(__file__)), "two_view.py")}
with contextlib.redirect_stdout(io.StringIO()):
    exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "two_view.py")).read(), ns)
views = ns["views"]
by = {(round(v["o"], 3), v["d"], v["ang"]): v for v in views}
D_ALL = sorted({v["d"] for v in views}); A_ALL = [90, 85, 80, 75, 70, 65, 60]
results = []
for dl in itertools.permutations([0.24, 0.26, 0.28], 3):
    for al in itertools.permutations([75, 70, 65, 60], 4):
        for o1, o2 in itertools.product((-0.04, -0.02, 0.0), (0.04, 0.06, 0.08)):
            pick = []
            for o in (o1, o2):
                ch = None
                for d in dl:
                    for an in al:
                        if (round(o, 3), d, an) in by:   # 노드에서 풀리고 관절 여유 · 상자 침범 통과한 것만 views 에 있음
                            ch = by[(round(o, 3), d, an)]; break
                    if ch: break
                pick.append(ch)
            if None in pick: continue
            v1, v2 = pick
            if all(x or y for x, y in zip(v1["mask"], v2["mask"])):
                mb1 = min(max(p1[0] if m1 else -1e9, p2[0] if m2 else -1e9) for p1, p2, m1, m2 in zip(v1["per"], v2["per"], v1["mask"], v2["mask"]))
                mb2 = min(max(p1[1] if m1 else -1e9, p2[1] if m2 else -1e9) for p1, p2, m1, m2 in zip(v1["per"], v2["per"], v1["mask"], v2["mask"]))
                results.append((min(mb1 / 30, mb2 / 16, v1["jm"] / .1, v2["jm"] / .1), dl, al, o1, o2, v1, v2, mb1, mb2))
results.sort(key=lambda t: -t[0])
print("목록 하나로 9 칸 덮는 경우", len(results))
for sc, dl, al, o1, o2, v1, v2, mb1, mb2 in results[:6]:
    print(f"빠듯한 비 {sc:.2f} 거리 {list(dl)} 각 {list(al)} | 보는 점 o {o1:+.2f}/{o2:+.2f} → {v1['d']:.2f} {v1['ang']:.0f}° · {v2['d']:.2f} {v2['ang']:.0f}° | B1 {mb1:.0f} px · B2 {mb2:.1f} px")
