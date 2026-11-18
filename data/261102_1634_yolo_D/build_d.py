"""후보 D 자료 만들기(11-02): 후보 C 자료(B 학습 자료 + 튜터 154225 train · val, 심볼릭 링크) + 실물 근접 사진(사용자 고친 라벨, 261102_1330_yolo_close_draft).
실물은 촬영 묶음 단위로 나눔: 검증(val) = f1223 · r1223 · a1244(학습 중 확인용, 가중치는 last.pt 고정), 나머지는 학습(train)에 2 번 넣음(실물 비중 약 38 %).
판정은 새로 찍는 영상으로 함(이 사진은 판정에 안 씀)."""
import os
from pathlib import Path

C = Path('/home/pa4/git/lv2_next_module/261102_1141_tutor_yolo/C_tutor')
R = Path('/home/pa4/git/lv2_next_module/261102_1330_yolo_close_draft')
D = Path(__file__).resolve().parent / 'D_real'
VAL = ('f1223', 'r1223', 'a1244')
REPEAT = 2
n = {}
for split in ('train', 'val'):
    (D / 'images' / split).mkdir(parents=True, exist_ok=False)
    (D / 'labels' / split).mkdir(parents=True, exist_ok=False)
    for img in sorted((C / 'images' / split).iterdir()):
        os.symlink(os.path.realpath(img), D / 'images' / split / img.name)
        (D / 'labels' / split / (img.stem + '.txt')).write_text((C / 'labels' / split / (img.stem + '.txt')).read_text())
        n[(split, 'C')] = n.get((split, 'C'), 0) + 1
for folder in ('real_labels_close_front_C', 'real_labels_close_rear_C', 'real_labels_arm720_C'):
    for img in sorted((R / folder / 'images').iterdir()):
        split = 'val' if img.name[:5] in VAL else 'train'
        lab = (R / folder / 'labels' / (img.stem + '.txt')).read_text()
        for k in range(1 if split == 'val' else REPEAT):
            name = f'real{k}_{img.name}'
            os.symlink(os.path.realpath(img), D / 'images' / split / name)
            (D / 'labels' / split / (Path(name).stem + '.txt')).write_text(lab)
            n[(split, folder[12:17])] = n.get((split, folder[12:17]), 0) + 1
(D / 'data.yaml').write_text(f"path: {D}\ntrain: images/train\nval: images/val\nnames:\n  0: red_cylinder\n  1: green_cylinder\n  2: red_cuboid\n  3: green_cuboid\n")
print(n)
