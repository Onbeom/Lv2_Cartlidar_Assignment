#!/usr/bin/env python3
"""영상 원본 모든 프레임에서 tag36h11 을 전체 해상도로 찾아 코너(2D)만 저장한다. 내부값이 필요 없다.
실행 (PC): ~/yolo-venv/bin/python tag_scan.py <영상 폴더(…/video)> <글롭> <출력 jsonl>
출력 줄: {"video","frame","tags":[{"id","c":[[x,y]*4],"side_px","sharp"}]}
"""
import glob, json, os, sys
from multiprocessing import Pool
import cv2, numpy as np

P = cv2.aruco.DetectorParameters()
P.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
P.adaptiveThreshWinSizeMin, P.adaptiveThreshWinSizeMax, P.adaptiveThreshWinSizeStep = 3, 53, 10
P.minMarkerPerimeterRate = 0.012
DET = None

def work(f):
    global DET
    if DET is None:
        cv2.setNumThreads(1)
        DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), P)
    g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    c, i, _ = DET.detectMarkers(g)
    tags = []
    if i is not None:
        for q, k in zip(c, i.ravel()):
            if k > 14: continue            # 과제 태그는 0~14
            q = q.reshape(4, 2)
            s = float(np.mean(np.linalg.norm(q - np.roll(q, 1, 0), axis=1)))
            x0, y0 = np.maximum(q.min(0).astype(int) - 2, 0); x1, y1 = q.max(0).astype(int) + 3
            sh = float(cv2.Laplacian(g[y0:y1, x0:x1], cv2.CV_64F).var())
            tags.append(dict(id=int(k), c=np.round(q, 2).tolist(), side_px=round(s, 1), sharp=round(sh, 0)))
    return f, tags

if __name__ == '__main__':
    root, pat, out = sys.argv[1:4]
    fs = []
    for d in sorted(glob.glob(os.path.join(root, pat))):
        fs += sorted(glob.glob(d + '/arm/frames/*.jpg'))
    print(len(fs), '프레임', flush=True)
    n = t = 0
    with Pool(8) as p, open(out, 'w') as w:
        for f, tags in p.imap(work, fs, chunksize=32):
            n += 1; t += bool(tags)
            v = f.split('/')[-4]
            w.write(json.dumps(dict(video=v, frame=os.path.basename(f), tags=tags), ensure_ascii=False) + '\n')
            if n % 1000 == 0: print(n, '태그 있는 프레임', t, flush=True)
    print('끝', n, t, flush=True)
