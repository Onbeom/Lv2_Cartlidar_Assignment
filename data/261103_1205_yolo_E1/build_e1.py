"""후보 E1 자료(11-03): D 자료(심볼릭 링크 그대로) + D 학습용 실물 사진(real0_*, 1048장)마다 오프라인 증강 2벌.
① 움직임 흐림: 무작위 방향 선 커널 길이 7~21 px(영상 판정에서 놓친 장이 흐린 장에 몰림 — 261103_1053_video_eval)
② 햇빛 과노출: 밝기 × 1.4~2.0 + 감마 0.6~0.8(앞 D435 front_v2 · v3 햇빛 장면에서 놓침)
라벨은 원본 그대로(기하 변화 없음). 증강 장은 JPEG 95(디스크 절약). 판정 영상(11-03)은 학습에 쓰지 않음."""
import os, random
from pathlib import Path
import cv2, numpy as np

D = Path('/home/pa4/git/lv2_next_module/261102_1634_yolo_D/D_real'); E = Path(__file__).resolve().parent / 'E1_aug'
random.seed(0); np.random.seed(0); n = {}
for split in ('train', 'val'):
    (E / 'images' / split).mkdir(parents=True, exist_ok=False); (E / 'labels' / split).mkdir(parents=True, exist_ok=False)
    for img in sorted((D / 'images' / split).iterdir()):
        os.symlink(os.path.realpath(img), E / 'images' / split / img.name)
        (E / 'labels' / split / (img.stem + '.txt')).write_text((D / 'labels' / split / (img.stem + '.txt')).read_text())
        n[(split, 'D')] = n.get((split, 'D'), 0) + 1


def blur(im):
    L = random.randint(7, 21); k = np.zeros((L, L), np.float32); k[L // 2, :] = 1
    M = cv2.getRotationMatrix2D((L / 2 - 0.5, L / 2 - 0.5), random.uniform(0, 180), 1); k = cv2.warpAffine(k, M, (L, L)); k /= k.sum()
    return cv2.filter2D(im, -1, k)


def sun(im):
    g, gam = random.uniform(1.4, 2.0), random.uniform(0.6, 0.8)
    x = np.clip(im.astype(np.float32) / 255 * g, 0, 1) ** gam
    return (x * 255).astype(np.uint8)


for img in sorted((D / 'images' / 'train').glob('real0_*')):
    im = cv2.imread(os.path.realpath(img)); lab = (D / 'labels' / 'train' / (img.stem + '.txt')).read_text()
    for tag, f in (('blur', blur), ('sun', sun)):
        name = f'aug{tag}_{img.stem[6:]}.jpg'
        cv2.imwrite(str(E / 'images' / 'train' / name), f(im), [cv2.IMWRITE_JPEG_QUALITY, 95])
        (E / 'labels' / 'train' / (Path(name).stem + '.txt')).write_text(lab); n[('train', tag)] = n.get(('train', tag), 0) + 1
(E / 'data.yaml').write_text(f"path: {E}\ntrain: images/train\nval: images/val\nnames:\n  0: red_cylinder\n  1: green_cylinder\n  2: red_cuboid\n  3: green_cuboid\n")
print(n)
