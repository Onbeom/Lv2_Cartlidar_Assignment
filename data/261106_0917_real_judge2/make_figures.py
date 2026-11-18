#!/usr/bin/env python3
"""판정 2차 그림 (PC): 입력 처리 그림의 공통 함수. 라벨 찾기는 barcode_reader.cpp read_labels 를 같은 값으로 옮긴 것(그림용, 판정 결과는 C++ 결과만 쓴다)."""
import cv2
import numpy as np


def find_labels(gray, rel=0.85, min_area=0.004, max_area=0.5, ar_min=1.8, ar_max=6.0, fill_min=0.75, max_labels=3):
    b = cv2.GaussianBlur(gray, (5, 5), 0)
    hist = np.bincount(b.ravel(), minlength=256)
    p995 = int(np.searchsorted(np.cumsum(hist), 0.995 * b.size))
    _, bw = cv2.threshold(b, rel * p995, 255, cv2.THRESH_BINARY)
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cand = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < min_area * gray.size or a > max_area * gray.size:
            continue
        (_, (w, h), _) = cv2.minAreaRect(c)
        L, s = max(w, h), min(w, h)
        if s < 8 or L / s < ar_min or L / s > ar_max or a / (w * h) < fill_min:
            continue
        cand.append((a, c))
    cand.sort(key=lambda x: -x[0])
    return bw, p995, [c for _, c in cand[:max_labels]]


def rectify(gray, c, gamma=0.5, pad=0.35):
    hull = cv2.convexHull(c)
    ap = cv2.approxPolyDP(hull, 0.02 * cv2.arcLength(hull, True), True).reshape(-1, 2).astype(np.float32)
    if len(ap) == 4:
        m = ap.mean(0)
        q = sorted(ap.tolist(), key=lambda p: np.arctan2(p[1] - m[1], p[0] - m[0]))
        q = np.float32(q)
        e = lambda i: np.linalg.norm(q[(i + 1) % 4] - q[i])
        if e(0) + e(2) < e(1) + e(3):
            q = np.roll(q, -1, axis=0)
        W, H = int(round((e(0) + e(2)) / 2)), int(round((e(1) + e(3)) / 2))
        crop = cv2.warpPerspective(gray, cv2.getPerspectiveTransform(q, np.float32([[0, 0], [W, 0], [W, H], [0, H]])), (W, H), borderValue=255)
    else:
        (cx, cy), (w, h), ang = cv2.minAreaRect(c)
        if w < h:
            w, h, ang = h, w, ang + 90
        r = cv2.warpAffine(gray, cv2.getRotationMatrix2D((cx, cy), ang, 1.0), gray.shape[::-1], borderValue=255)
        crop = r[max(0, round(cy - h / 2)):round(cy + h / 2), max(0, round(cx - w / 2)):round(cx + w / 2)]
    lut = np.round(255 * (np.arange(256) / 255.0) ** gamma).astype(np.uint8)
    g = cv2.LUT(crop, lut)
    py, px = int(pad * g.shape[0]) + 4, int(pad * g.shape[1]) + 4
    return crop, cv2.copyMakeBorder(g, py, py, px, px, cv2.BORDER_CONSTANT, value=255)


if __name__ == '__main__':
    import glob
    import json
    import os
    import subprocess
    import sys
    from PIL import Image, ImageDraw, ImageFont

    HERE = os.path.dirname(os.path.abspath(__file__))
    PHOTO = os.path.expanduser('~/git/lv2_next_module/photo')
    TAG_READ = os.path.expanduser('~/git/lv2_next_module/261105_2041_tag_detect_width/tag_read_tp')
    SCRATCH = sys.argv[1]
    FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc'
    f = lambda n: ImageFont.truetype(FONT, n)
    TAGS = {json.loads(l)['name']: json.loads(l) for l in open(f'{HERE}/results/tag.jsonl')}
    BARS = {json.loads(l)['name']: json.loads(l) for l in open(f'{HERE}/results/barcode.jsonl')}
    clip = lambda pre: sorted(glob.glob(f'{HERE}/clips/{pre}_10cm_*'))[0]

    def pil(a):
        return Image.fromarray(cv2.cvtColor(a, cv2.COLOR_BGR2RGB) if a.ndim == 3 else a).convert('RGB')

    def tile(a, w, h, cap, border=None):
        """a 를 w × h 안에 비율 유지로 넣고 위에 글자"""
        im = pil(a)
        r = min(w / im.width, h / im.height)
        im = im.resize((max(1, round(im.width * r)), max(1, round(im.height * r))), Image.LANCZOS if r < 1 else Image.NEAREST)
        cv = Image.new('RGB', (w, h + 30), (245, 245, 245))
        cv.paste(im, ((w - im.width) // 2, 30 + (h - im.height) // 2))
        d = ImageDraw.Draw(cv)
        d.text((4, 3), cap, font=f(17), fill=(20, 20, 20))
        if border:
            d.rectangle([0, 30, w - 1, h + 29], outline=border, width=5)
        return cv

    def page(rows, title_w):
        """rows: (제목 | None, [tile, ...])"""
        H = sum((40 if t else 0) + (max(x.height for x in r) + 8 if r else 0) for t, r in rows)
        out = Image.new('RGB', (title_w, H + 10), 'white')
        y, d = 5, ImageDraw.Draw(out)
        for t, r in rows:
            if t:
                d.text((8, y + 4), t, font=f(22), fill=(150, 30, 30))
                y += 40
            x = 4
            for c in r:
                out.paste(c, (x, y))
                x += c.width + 6
            if r:
                y += max(c.height for c in r) + 8
        return out

    def sharp(p):
        im = cv2.imread(p)
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        ys, xs = np.nonzero(((hsv[..., 0] < 12) | (hsv[..., 0] > 168)) & (hsv[..., 1] > 70))
        if len(xs) < 20:
            return None, None
        cx, cy = int(np.median(xs)), int(np.median(ys))
        s = int(max(np.percentile(xs, 98) - np.percentile(xs, 2), np.percentile(ys, 98) - np.percentile(ys, 2)) / 2 + 10)
        return cv2.Laplacian(g[max(0, cy - s):cy + s, max(0, cx - s):cx + s], cv2.CV_64F).var(), (cx, cy)

    def zoom(p, c, half=130, scale=2):
        im = cv2.imread(p)
        x0, y0 = max(0, min(c[0] - half, 1280 - 2 * half)), max(0, min(c[1] - half, 720 - 2 * half))
        return cv2.resize(im[y0:y0 + 2 * half, x0:x0 + 2 * half], None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

    # ---------- 그림 1: 미달 · 애매
    C3, C5, C6 = clip('tag03'), clip('tag05'), clip('tag06')
    n3 = os.path.basename(C3)
    VIS = set(range(51, 54)) | set(range(77, 96))
    still = [sharp(f'{C3}/{i:05d}.png')[0] for i in range(51)]
    miss = [i for i in sorted(VIS) if not TAGS[f'{n3}/{i:05d}'].get('ids')]
    print('정지 51장 선명도', round(min(still)), round(max(still)), '· 태그 보이는데 놓친', len(miss), '장 선명도',
          round(min(sharp(f'{C3}/{i:05d}.png')[0] for i in miss)), round(max(sharp(f'{C3}/{i:05d}.png')[0] for i in miss)))
    # 시간 띠
    bar = Image.new('RGB', (1560, 70), 'white')
    d = ImageDraw.Draw(bar)
    GREEN, RED, GREY = (40, 170, 70), (220, 50, 40), (170, 170, 170)
    for i in range(105):
        hit = bool(TAGS[f'{n3}/{i:05d}'].get('ids'))
        col = GREEN if hit else (RED if i in VIS else GREY)
        d.rectangle([i * 14 + 4, 2, i * 14 + 16, 34], fill=col)
    d.text((4, 40), '초록 = 읽음 53장 · 빨강 = 화면에 태그가 있는데 놓침 20장 · 회색 = 태그가 화면 밖 32장 (왼쪽부터 00000 → 00104)', font=f(18), fill=(30, 30, 30))
    bar_cv = Image.new('RGB', (1560, 70), 'white'); bar_cv.paste(bar, (0, 0))
    r1 = [tile(cv2.imread(f'{C3}/00050.png'), 385, 216, '00050 정지: 읽음', GREEN),
          tile(cv2.imread(f'{C3}/00053.png'), 385, 216, '00053 카메라 이동 시작: 놓침', RED),
          tile(cv2.imread(f'{C3}/00056.png'), 385, 216, '00056 태그 화면 밖', GREY),
          tile(cv2.imread(f'{C3}/00100.png'), 385, 216, '00100 태그 화면 밖', GREY)]
    r2 = []
    for i in (84, 86, 88, 91, 92, 87, 90):
        p = f'{C3}/{i:05d}.png'
        s, c = sharp(p)
        hit = bool(TAGS[f'{n3}/{i:05d}'].get('ids'))
        r2.append(tile(zoom(p, c), 188, 188, f'{i} {"읽음" if hit else "놓침"} 선명 {s:.0f}', GREEN if hit else RED))
    s50, c50 = sharp(f'{C3}/00050.png')
    r2.insert(0, tile(zoom(f'{C3}/00050.png', c50), 188, 188, f'00050 정지 읽음 선명 {s50:.0f}', GREEN))

    def boxed(path, frame_txt):
        im = cv2.imread(path)
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        _, _, ls = find_labels(g)
        cv2.drawContours(im, ls, -1, (0, 220, 255), 3)
        return im, g, ls
    im6, g6, l6 = boxed(f'{C6}/00060.png', '')
    im5, _, _ = boxed(f'{C5}/00118.png', '')
    cr6 = cv2.imread(f'{C6}/00060.png')[0:230, 960:1280]
    cr5 = cv2.imread(f'{C5}/00118.png')[0:230, 960:1280]
    r4 = [tile(im6, 512, 288, 'tag06 00060: 오른쪽 위 라벨을 P2-03 으로 읽음'),
          tile(cr6, 320, 230, 'tag06 라벨 확대'),
          tile(im5, 512, 288, 'tag05 00118: 같은 라벨 (1장만 읽음)'),
          tile(cr5, 200, 144, 'tag05 라벨 확대')]
    fig1 = page([('① tag03 T1 미달 (0.505): 앞 51장은 전부 읽음. 그 뒤 카메라가 움직임', [bar_cv]), (None, r1),
                 ('놓친 장 확대: 흔들림으로 칸이 찌그러짐 (선명 = 라플라시안 분산, 정지 51장은 %d~%d)' % (round(min(still)), round(max(still))), r2),
                 ('② B2 애매: tag06 130장 · tag05 1장에서 바코드를 읽음. 화면에 실제 라벨이 같이 찍혀 있음', r4)], 1570)
    fig1.save(f'{PHOTO}/261106_판정2차_미달과애매.png')

    # ---------- 그림 2: 입력 종류별
    def tag_row(pre, frame, title):
        cl = clip(pre)
        p, name = f'{cl}/{frame}.png', f'{os.path.basename(cl)}/{frame}'
        lst = f'{SCRATCH}/one.txt'
        open(lst, 'w').write(f'{name} {p}\n')
        out = subprocess.run([TAG_READ, lst, '640', 'corners'], capture_output=True, text=True).stdout
        r = json.loads(out.splitlines()[0])
        im = cv2.imread(p)
        g = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (640, 360), interpolation=cv2.INTER_AREA)
        for c in r.get('corners', []):
            cv2.polylines(im, [np.array(c, np.float32).reshape(4, 2).astype(np.int32)], True, (0, 255, 0), 3)
        t1 = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23, 7)
        t2 = cv2.adaptiveThreshold(g, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 23, 35)
        w, h = 300, 169
        rec = []
        if r.get('corners'):
            c = np.array(r['corners'][0], np.float32).reshape(4, 2)
            rec = cv2.warpPerspective(cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2GRAY), cv2.getPerspectiveTransform(c, np.float32([[0, 0], [160, 0], [160, 160], [0, 160]])), (160, 160))
        cells = [tile(im, w, h, f'1 원본 1280×720, ID {r["ids"]}'), tile(g, w, h, '2 회색 640×360 (찾는 입력)'),
                 tile(t1, w, h, '3 기본 문턱 (창 23 · 상수 7)'), tile(t2, w, h, '4 무늬 거르기 (창 23 · 상수 35)')]
        if len(rec):
            cells.append(tile(rec, 200, 169, '5 해독 칸 (펴서 봄)'))
        return (title, cells)

    def bar_row(pre, frame, title, want=0):
        cl = clip(pre)
        p = f'{cl}/{frame}.png'
        im = cv2.imread(p)
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        bw, p995, ls = find_labels(g)
        if not ls:
            return (title + ' (라벨 없음)', [tile(im, 300, 169, '원본')])
        cr, lab = rectify(g, ls[want])
        cv2.drawContours(im, ls, -1, (0, 220, 255), 3)
        rec = BARS.get(f'{os.path.basename(cl)}/{frame}', {})
        w, h = 300, 169
        return (title, [tile(im, w, h, f'1 원본, 읽은 값 {rec.get("text", "")}'), tile(bw, w, h, f'2 밝은 곳 찾기 (문턱 {0.85 * p995:.0f})'),
                        tile(cr, w, h, '3 라벨 잘라 펴기'), tile(lab, w, h, '4 감마 0.5 + 흰 여백 = zbar 입력')])

    rows = [bar_row(pre, '00050', f'바코드 {pre}') for pre in ('P2-01', 'P2-02', 'P2-03', 'P2-04')]
    rows.append(tag_row('tag03', '00030', '퍽 태그 (tag03, 원기둥)'))
    rows.append(tag_row('tag06', '00150', '퍽 태그 (tag06, 직육면체)'))
    rows += [tag_row(pre, '00050', f'장소 태그 {pre}') for pre in ('L1', 'L2', 'L3')]
    page(rows, 1460).save(f'{PHOTO}/261106_판정2차_입력종류별.png')
