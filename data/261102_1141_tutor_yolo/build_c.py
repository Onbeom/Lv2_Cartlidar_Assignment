"""후보 C 자료 만들기(11-02): 후보 B 학습 자료(261030_1325/B_negative train · val) + 튜터 합성 tagged-practice-20261029-154225 train · val(가까운 태그 퍽, 클래스 바꿈).
이미지는 심볼릭 링크(디스크 여유 14 GB), 라벨은 새로 씀. 튜터 test(seed 5000~5499, train 1000~2499 · val 3000~3299 와 겹치지 않음)는 평가용으로만 남김.
튜터 152648 · 152001 은 seed 범위가 154225 와 같아(같은 장면일 수 있음) 쓰지 않음."""
import os
from pathlib import Path

B = Path('/home/pa4/git/lv2_next_module/261030_1325/B_negative')
T = Path("/home/pa4/git/lv2_next_module/tutor's/yolov8-isaacsim-data/tagged-practice-20261029-154225")
C = Path(__file__).resolve().parent / 'C_tutor'
MAP = {0: 0, 1: 2, 2: 1, 3: 3}     # 튜터 red_cylinder · red_prism · green_cylinder · green_prism → 우리 순서
n = {}
for split in ('train', 'val'):
    (C / 'images' / split).mkdir(parents=True, exist_ok=False)
    (C / 'labels' / split).mkdir(parents=True, exist_ok=False)
    for img in sorted((B / 'images' / split).iterdir()):
        os.symlink(img.resolve(), C / 'images' / split / img.name)
        lab = B / 'labels' / split / (img.stem + '.txt')
        (C / 'labels' / split / (img.stem + '.txt')).write_text(lab.read_text() if lab.exists() else '')
        n[(split, 'B')] = n.get((split, 'B'), 0) + 1
    for img in sorted((T / 'images' / split).iterdir()):
        name = 'tutor154225_' + img.name
        os.symlink(img.resolve(), C / 'images' / split / name)
        lab = T / 'labels' / split / (img.stem + '.txt')
        lines = []
        for line in (lab.read_text().split('\n') if lab.exists() else []):
            p = line.split()
            if len(p) == 5:
                lines.append(' '.join([str(MAP[int(p[0])])] + p[1:]))
        (C / 'labels' / split / (Path(name).stem + '.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''))
        n[(split, 'tutor')] = n.get((split, 'tutor'), 0) + 1
(C / 'data.yaml').write_text(f"path: {C}\ntrain: images/train\nval: images/val\nnames:\n  0: red_cylinder\n  1: green_cylinder\n  2: red_cuboid\n  3: green_cuboid\n")
print(n)
