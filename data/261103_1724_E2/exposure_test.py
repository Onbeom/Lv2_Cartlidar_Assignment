"""노출 테스트(11-03 옆방 · 경기장과 같은 조명 · 커튼 내림 조건, 사람이 손으로 움직임, 판정 아님 — 운용 노출 추천용).
같은 배치에서 앞 · 뒤 D435 를 자동 노출(시작 166 · gain 64) / 노출 80 · gain 128 / 노출 40 · gain 128 로 90초씩 찍은 영상의 모든 장에 대해:
선명도(라플라시안 분산, 클수록 덜 흐림) · 밝기(회색 평균) · 퍼센트 날아감(255) · D 0.6 이상 검출 수 · 검출 점수 중앙값.
배치가 같으므로 장당 0.6 이상 검출 수가 많을수록(같은 움직임이면) 잘 보는 것. 라벨 없음. 사용: exposure_test.py <영상 폴더>"""
import json, os, sys
from pathlib import Path
from collections import defaultdict
import cv2, numpy as np
os.environ['YOLO_OFFLINE'] = 'true'
from ultralytics import YOLO
m = YOLO('/home/pa4/git/lv2_next_module/261102_1634_yolo_D/yolo_export/yolo11n_task4_D.pt'); dev = os.environ.get('DEV', 'cpu')
R = {}
for d in sorted(p for p in Path(sys.argv[1]).iterdir() if (p / 'capture.json').exists()):
    lab = json.load(open(d / 'capture.json'))['label']; imgs = sorted((d / 'color').glob('*.png'))
    sh, br, sat, n6, sc = [], [], [], [], []
    for k in range(0, len(imgs), 32):
        batch = [cv2.imread(str(p)) for p in imgs[k:k + 32]]
        for im in batch:
            g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY); sh.append(cv2.Laplacian(g, cv2.CV_64F).var()); br.append(g.mean()); sat.append((g >= 250).mean())
        for r in m.predict(batch, conf=0.6, iou=0.45, imgsz=640, device=dev, verbose=False):
            n6.append(len(r.boxes)); sc += [float(x) for x in r.boxes.conf]
    R[lab] = {'frames': len(imgs), 'sharp_p50': round(float(np.median(sh)), 1), 'sharp_p25': round(float(np.percentile(sh, 25)), 1),
              'bright_mean': round(float(np.mean(br)), 1), 'saturated_pct': round(float(np.mean(sat)) * 100, 2),
              'det06_per_frame': round(float(np.mean(n6)), 2), 'frames_with_det06_pct': round(float(np.mean([x > 0 for x in n6])) * 100, 1),
              'score_p50': round(float(np.median(sc)), 3) if sc else None}
    print(lab, R[lab], flush=True)
json.dump(R, open(Path(__file__).with_name('exposure_test.json'), 'w'), ensure_ascii=False, indent=1)
