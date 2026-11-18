"""후보 E2 자료(11-03): D 자료(심볼릭 링크) + 11-03 실물 영상 장(사용자 고친 라벨)을 학습에 더함. 흔들림 증강 없음(사용자: 주행 시험 뒤).
- 오전 판정 영상 D435(video_front · video_rear, 540장, 햇빛 · 흔들림) · 커튼 판정 영상 D435(judge_front · judge_rear, 540장 — B·C·D 고르는 데 이미 썼으므로 학습으로 돌리고 E2 최종 판정은 새 경기장 영상으로)
  · 누운 퍽 학습 영상(train2_front · train2_rear, 270장, 노출 자동/80/40).
- 같은 퍽에 거의 같은 상자 둘(IoU ≥ 0.9, 라벨 실수)은 앞의 것 하나만 남김(학습에 서로 다른 클래스 두 개가 겹쳐 들어가지 않게).
- 실물 새 장은 2번 넣음(D 와 같은 방식). Arducam 판정 세트(video_arm)는 판정용으로 남겨 학습에 넣지 않음."""
import os
from pathlib import Path
D = Path('/home/pa4/git/lv2_next_module/261102_1634_yolo_D/D_real'); V = Path('/home/pa4/git/lv2_next_module/261103_1053_video_eval')
HERE = Path(__file__).resolve().parent; E = HERE / 'E2_data'
SRC = [(V, 'video_front'), (V, 'video_rear'), (V, 'judge_front'), (V, 'judge_rear'), (HERE, 'train2_front'), (HERE, 'train2_rear')]
REPEAT = 2; n = {}; dup = 0


def iou(a, b):
    ax0, ay0, ax1, ay1 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx0, by0, bx1, by1 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    ix = max(0, min(ax1, bx1) - max(ax0, bx0)); iy = max(0, min(ay1, by1) - max(ay0, by0)); u = a[2] * a[3] + b[2] * b[3] - ix * iy
    return ix * iy / u if u > 0 else 0


for split in ('train', 'val'):
    (E / 'images' / split).mkdir(parents=True, exist_ok=False); (E / 'labels' / split).mkdir(parents=True, exist_ok=False)
    for img in sorted((D / 'images' / split).iterdir()):
        os.symlink(os.path.realpath(img), E / 'images' / split / img.name)
        (E / 'labels' / split / (img.stem + '.txt')).write_text((D / 'labels' / split / (img.stem + '.txt')).read_text())
        n[(split, 'D')] = n.get((split, 'D'), 0) + 1
for root, folder in SRC:
    for img in sorted((root / folder / 'images').iterdir()):
        L = [l.split() for l in (root / folder / 'labels' / (img.stem + '.txt')).read_text().split('\n') if l.strip()]
        keep = []
        for p in L:
            b = list(map(float, p[1:]))
            if any(iou(b, list(map(float, q[1:]))) >= 0.9 for q in keep): dup += 1; continue
            keep.append(p)
        lab = ''.join(' '.join(p) + '\n' for p in keep)
        for k in range(REPEAT):
            name = f'v{k}_{folder}__{img.name}'
            os.symlink(os.path.realpath(img), E / 'images' / 'train' / name); (E / 'labels' / 'train' / (Path(name).stem + '.txt')).write_text(lab)
        n[('train', folder)] = n.get(('train', folder), 0) + 1
(E / 'data.yaml').write_text(f"path: {E}\ntrain: images/train\nval: images/val\nnames:\n  0: red_cylinder\n  1: green_cylinder\n  2: red_cuboid\n  3: green_cuboid\n")
print(n, '겹친 상자 뺌', dup)
