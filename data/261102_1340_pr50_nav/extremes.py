"""footprint_compare.py 와 같은 FK 로 링크별 x · |y| 범위와 z 범위를 표로(바깥쪽 링크 찾기)."""
import runpy, sys
import numpy as np
g = runpy.run_path(str(__import__('pathlib').Path(__file__).with_name('footprint_compare.py')))
rows = []
for n, vs in g['pts'].items():
    v = np.vstack(vs); rows.append((n, v[:, 0].min(), v[:, 0].max(), np.abs(v[:, 1]).max(), v[:, 2].min(), v[:, 2].max()))
for key, rev in ((1, False), (2, True), (3, True)):
    print(['', 'x 최소', 'x 최대', '|y| 최대'][key], *[f'{r[0]}[{r[key]:.4f}, z {r[4]:.3f}~{r[5]:.3f}]' for r in sorted(rows, key=lambda r: r[key], reverse=rev)[:4]])
