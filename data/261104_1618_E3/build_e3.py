"""후보 E3 자료(11-04): E2 자료 전부(심볼릭 링크 · 라벨 복사) + 팔 근접 학습 영상 train3_arm(사용자 고친 라벨) 3번.
- train3_arm: Arducam 1280×720, 0.5 m → 집기 직전 근접 · 위에서 본 면 · 누운 퍽 · 켜진 신호등 배경(v5), 1초 1장 257장.
- 3번 넣는 이유: E2 의 Arducam 장은 D 때 908장(× 2)뿐이고 경기장 판정에서 가까운 퍽을 놓침 → 새 근접 장 비중을 높임(실물 D435 새 장은 E2 에서 2번).
- 같은 퍽에 거의 같은 상자 둘(IoU ≥ 0.9)은 앞의 것만. 판정 세트(261104_1211_arena_eval 전부, 11-03 video_arm)는 넣지 않음."""
import os
from pathlib import Path
HERE = Path(__file__).resolve().parent; E2 = Path('/home/pa4/git/lv2_next_module/261103_1724_E2/E2_data'); E = HERE / 'E3_data'
REPEAT = 3; n = {}; dup = 0


def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax1, bx1) - max(ax0, bx0)); iy = max(0, min(ay1, by1) - max(ay0, by0)); u = a[2] * a[3] + b[2] * b[3] - ix * iy
    return ix * iy / u if u > 0 else 0


for split in ('train', 'val'):
    (E / 'images' / split).mkdir(parents=True, exist_ok=False); (E / 'labels' / split).mkdir(parents=True, exist_ok=False)
    for img in sorted((E2 / 'images' / split).iterdir()):
        os.symlink(os.path.realpath(img), E / 'images' / split / img.name)
        (E / 'labels' / split / (img.stem + '.txt')).write_text((E2 / 'labels' / split / (img.stem + '.txt')).read_text())
        n[(split, 'E2')] = n.get((split, 'E2'), 0) + 1
src = HERE / 'train3_arm'
for img in sorted((src / 'images').iterdir()):
    L = [l.split() for l in (src / 'labels' / (img.stem + '.txt')).read_text().split('\n') if l.strip()]; keep = []
    for i, p in enumerate(L):  # 화면 밖으로 아주 조금 나간 상자(높이 1.000001 등, 사용자 도구 반올림)를 0~1 로 자름
        x, y, w, h = map(float, p[1:]); x0, y0, x1, y1 = max(0, x - w / 2), max(0, y - h / 2), min(1, x + w / 2), min(1, y + h / 2)
        L[i] = [p[0]] + [f'{v:.6f}' for v in ((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)]
    for p in L:
        if any(iou(list(map(float, p[1:])), list(map(float, q[1:]))) >= 0.9 for q in keep): dup += 1; continue
        keep.append(p)
    lab = ''.join(' '.join(p) + '\n' for p in keep)
    for k in range(REPEAT):
        name = f'a{k}_train3_arm__{img.name}'
        os.symlink(os.path.realpath(img), E / 'images' / 'train' / name); (E / 'labels' / 'train' / (Path(name).stem + '.txt')).write_text(lab)
    n[('train', 'train3_arm')] = n.get(('train', 'train3_arm'), 0) + 1
(E / 'data.yaml').write_text(f"path: {E}\ntrain: images/train\nval: images/val\nnames:\n  0: red_cylinder\n  1: green_cylinder\n  2: red_cuboid\n  3: green_cuboid\n")
print(n, '겹친 상자 뺌', dup)
