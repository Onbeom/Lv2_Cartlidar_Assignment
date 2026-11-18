"""실물 라벨 초안 고치기(사람용 도구): auto_label.py 가 만든 <라벨 폴더>/images · labels 를 한 장씩 보며 상자를 지우고 · 그리고 · 클래스를 바꾼다.

실행 (PC): ~/yolo-venv/bin/python Cartlidar/yolo/fix_labels.py --dir 261026_1414/real_labels [--batch r01] [--all] [--images 107,110]
  기본은 아직 안 본 장(checked=false)부터 연다. --all 이면 처음부터 전부.

마우스: 왼쪽 끌기 = 새 상자(지금 클래스) · 왼쪽 클릭 = 상자 고르기 · 고른 상자의 모서리 끌기 = 크기 조절 · 오른쪽 클릭 = 그 상자 지우기
키: 1 red_cylinder · 2 green_cylinder · 3 red_cuboid · 4 green_cuboid (고른 상자의 클래스를 바꾸고, 새 상자 클래스도 됨,
    번호는 <라벨 폴더>/classes.txt 를 따름 — 6클래스 · 4클래스 모두)
    d = 고른 상자 지우기 · z = 되돌리기 · e = 이 장 상자 모두 지우기(빈 장면) · b = 화면만 밝게(저노출 사진용, 파일은 그대로)
    스페이스 / n = 저장 · '다 봄' 표시 · 다음 장 · p = 저장 · 앞 장 · s = 저장 · q / Esc = 저장 · 끝
저장할 때 처음 초안과 비교해 지움 · 추가 · 클래스 바꿈 · 크기 고침 수를 auto_label.json 의 edits 에 적는다(시뮬레이션 모델의 실물 성능 대략 확인용).
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

import puck_classes as pc

# 클래스 번호는 <라벨 폴더>/classes.txt 이름으로 정함(6클래스 · 4클래스 모두, 없으면 6클래스). main() 에서 채움.
NAMES = KEY_CLASS = COLOR = IS_CYL = None


def set_classes(names):
    global NAMES, KEY_CLASS, COLOR, IS_CYL
    NAMES = names
    # 키 1 · 2 · 3 · 4 = red_cylinder · green_cylinder · red_cuboid · green_cuboid(파랑은 과제에서 빠짐)
    KEY_CLASS = {ord(str(i + 1)): names.index(n) for i, n in enumerate(pc.NAMES4)}
    COLOR = {i: pc.BGR[pc.split(n)[0]] for i, n in enumerate(names)}
    IS_CYL = {i: pc.split(n)[1] == "cylinder" for i, n in enumerate(names)}
HANDLE = 7  # 모서리 잡는 거리(화면 화소)


def read_boxes(p, w, h):
    """YOLO 줄 → [클래스, x0, y0, x1, y1] (원본 화소)."""
    out = []
    for ln in p.read_text().split("\n") if p.exists() else []:
        if ln.strip():
            k, x, y, bw, bh = ln.split()
            x, y, bw, bh = float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
            out.append([int(k), x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2])
    return out


def to_lines(boxes, w, h):
    return [f"{k} {(x0 + x1) / 2 / w:.6f} {(y0 + y1) / 2 / h:.6f} {(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}"
            for k, x0, y0, x1, y1 in boxes]


def iou(a, b):
    ix = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    iy = max(0.0, min(a[4], b[4]) - max(a[2], b[2]))
    inter = ix * iy
    union = (a[3] - a[1]) * (a[4] - a[2]) + (b[3] - b[1]) * (b[4] - b[2]) - inter
    return inter / union if union > 0 else 0.0


def count_edits(draft, boxes):
    """초안과 지금 상자를 IoU ≥ 0.5 로 짝지어 지움 · 추가 · 클래스 바꿈 · 크기 고침(짝 IoU < 0.9) 수를 센다."""
    used, cls, resized = set(), 0, 0
    for b in boxes:
        best = max(((iou(d, b), i) for i, d in enumerate(draft) if i not in used), default=(0.0, -1))
        if best[0] >= 0.5:
            used.add(best[1])
            cls += draft[best[1]][0] != b[0]
            resized += best[0] < 0.9
    matched = len(used)
    return {"deleted": len(draft) - matched, "added": len(boxes) - matched, "class_changed": cls, "resized": resized}


class Editor:
    def __init__(self, root, entries, scale):
        self.root, self.entries, self.scale = root, entries, scale
        self.cur_class, self.bright, self.sel = NAMES.index("green_cylinder"), False, None
        self.drag = None  # ("new", x, y) 또는 ("corner", 상자 번호, 모서리 번호)
        self.tmp = None

    # ---- 한 장 열기 · 저장 ----
    def load(self, i):
        self.i = i
        e = self.entries[i]
        self.img = cv2.imread(str(self.root / "images" / e["image"]))
        self.h, self.w = self.img.shape[:2]
        self.lbl = self.root / "labels" / (Path(e["image"]).stem + ".txt")
        self.boxes = read_boxes(self.lbl, self.w, self.h)
        if "draft" not in e:  # 처음 열 때 초안을 남겨 둠(고친 수 세기용)
            e["draft"] = to_lines(self.boxes, self.w, self.h)
        self.history, self.sel = [], None

    def save(self, checked=None):
        e = self.entries[self.i]
        lines = to_lines(self.boxes, self.w, self.h)
        self.lbl.write_text("".join(ln + "\n" for ln in lines))
        draft = read_boxes_from_lines(e["draft"], self.w, self.h)
        e["edits"] = count_edits(draft, self.boxes)
        e["dets"] = len(self.boxes)
        if checked is not None:
            e["checked"] = checked
        self.write_json()

    def write_json(self):
        (self.root / "auto_label.json").write_text(json.dumps(self.all_entries, indent=1, ensure_ascii=False))

    # ---- 편집 ----
    def push(self):
        self.history.append([b[:] for b in self.boxes])

    def hit(self, x, y):
        """점 (원본 화소)을 품은 상자 중 가장 작은 것."""
        inside = [(b[3] - b[1]) * (b[4] - b[2]) for b in self.boxes]
        cands = [i for i, b in enumerate(self.boxes) if b[1] <= x <= b[3] and b[2] <= y <= b[4]]
        return min(cands, key=lambda i: inside[i]) if cands else None

    def corner(self, x, y):
        if self.sel is None:
            return None
        k, x0, y0, x1, y1 = self.boxes[self.sel]
        r = HANDLE / self.scale
        for n, (cx, cy) in enumerate([(x0, y0), (x1, y0), (x0, y1), (x1, y1)]):
            if abs(x - cx) <= r and abs(y - cy) <= r:
                return n
        return None

    def mouse(self, ev, dx, dy, flags, _):
        x, y = dx / self.scale, dy / self.scale
        x, y = min(max(x, 0), self.w - 1), min(max(y, 0), self.h - 1)
        if ev == cv2.EVENT_LBUTTONDOWN:
            c = self.corner(x, y)
            if c is not None:
                self.push()
                self.drag = ("corner", self.sel, c)
            else:
                self.drag = ("new", x, y)
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag:
            if self.drag[0] == "corner":
                b = self.boxes[self.drag[1]]
                c = self.drag[2]
                b[1 if c in (0, 2) else 3] = x
                b[2 if c in (0, 1) else 4] = y
            else:
                self.tmp = (self.drag[1], self.drag[2], x, y)
        elif ev == cv2.EVENT_LBUTTONUP and self.drag:
            if self.drag[0] == "corner":
                b = self.boxes[self.drag[1]]
                b[1], b[3] = sorted((b[1], b[3]))
                b[2], b[4] = sorted((b[2], b[4]))
            else:
                x0, y0 = self.drag[1], self.drag[2]
                if abs(x - x0) * self.scale >= 5 and abs(y - y0) * self.scale >= 5:
                    self.push()
                    self.boxes.append([self.cur_class, min(x0, x), min(y0, y), max(x0, x), max(y0, y)])
                    self.sel = len(self.boxes) - 1
                else:
                    self.sel = self.hit(x, y)
            self.drag, self.tmp = None, None
        elif ev == cv2.EVENT_RBUTTONDOWN:
            j = self.hit(x, y)
            if j is not None:
                self.push()
                del self.boxes[j]
                self.sel = None

    def key(self, k):
        if k in KEY_CLASS:
            self.cur_class = KEY_CLASS[k]
            if self.sel is not None and self.boxes[self.sel][0] != self.cur_class:
                self.push()
                self.boxes[self.sel][0] = self.cur_class
        elif k == ord("d") and self.sel is not None:
            self.push()
            del self.boxes[self.sel]
            self.sel = None
        elif k == ord("z") and self.history:
            self.boxes, self.sel = self.history.pop(), None
        elif k == ord("e") and self.boxes:
            self.push()
            self.boxes, self.sel = [], None
        elif k == ord("b"):
            self.bright = not self.bright

    # ---- 그리기 ----
    def draw(self):
        im = self.img
        if self.bright:
            im = np.clip((im.astype(np.float32) / 255) ** 0.4 * 255, 0, 255).astype(np.uint8)
        im = cv2.resize(im, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)
        s = self.scale
        for n, (k, x0, y0, x1, y1) in enumerate(self.boxes):
            p0, p1 = (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s))
            col = (255, 255, 0) if n == self.sel else COLOR[k]
            cv2.rectangle(im, p0, p1, col, 1 if IS_CYL[k] else 2)  # 얇은 선 원기둥 · 굵은 선 직육면체
            cv2.putText(im, ("cyl" if IS_CYL[k] else "cub"), (p0[0], max(10, p0[1] - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)
            if n == self.sel:
                for c in [(p0[0], p0[1]), (p1[0], p0[1]), (p0[0], p1[1]), (p1[0], p1[1])]:
                    cv2.circle(im, c, HANDLE - 2, col, 1)
        if self.tmp:
            x0, y0, x1, y1 = self.tmp
            cv2.rectangle(im, (int(x0 * s), int(y0 * s)), (int(x1 * s), int(y1 * s)), COLOR[self.cur_class], 1)
        e = self.entries[self.i]
        top = (f"{self.i + 1}/{len(self.entries)} {e['batch']}  boxes {len(self.boxes)}  new={NAMES[self.cur_class]}"
               f"  {'CHECKED' if e.get('checked') else 'unchecked'}{'  [bright]' if self.bright else ''}")
        cv2.rectangle(im, (0, 0), (im.shape[1], 20), (255, 255, 255), -1)
        cv2.putText(im, top, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return im


def read_boxes_from_lines(lines, w, h):
    out = []
    for ln in lines:
        k, x, y, bw, bh = ln.split()
        x, y, bw, bh = float(x) * w, float(y) * h, float(bw) * w, float(bh) * h
        out.append([int(k), x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, required=True, help="auto_label.py 의 --out 폴더")
    ap.add_argument("--batch", help="이 글자로 시작하는 묶음만 (예: r01)")
    ap.add_argument("--all", action="store_true", help="다 본 장도 처음부터 연다")
    ap.add_argument("--images", help="이 사진 번호만 연다(쉼표로, 예: 107,110). 다 본 장도 연다")
    ap.add_argument("--scale", type=float, default=2.0, help="화면 확대 배율")
    args = ap.parse_args()
    set_classes(pc.read_names(args.dir / "classes.txt"))
    all_entries = json.loads((args.dir / "auto_label.json").read_text())
    entries = [e for e in all_entries if not args.batch or e["batch"].startswith(args.batch)]
    if args.images:
        nums = {n.strip().zfill(3) for n in args.images.split(",")}
        entries = [e for e in entries if e["image"].split("__", 1)[-1][:3] in nums]
    if not entries:
        raise SystemExit("고를 장이 없음")
    start = 0 if args.all or args.images else next((i for i, e in enumerate(entries) if not e.get("checked")), None)
    if start is None:
        raise SystemExit("모두 다 봄(--all 로 다시 열 수 있음)")
    ed = Editor(args.dir, entries, args.scale)
    ed.all_entries = all_entries
    ed.load(start)
    win = "fix_labels"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(win, ed.mouse)
    print(__doc__.split("\n\n", 1)[1])
    while True:
        cv2.imshow(win, ed.draw())
        k = cv2.waitKey(30) & 0xFF
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:  # 창 닫기 단추
            ed.save()
            break
        if k in (ord("q"), 27):
            ed.save()
            break
        if k in (ord(" "), ord("n")):
            ed.save(checked=True)
            if ed.i + 1 >= len(entries):
                print("마지막 장까지 봄")
                break
            ed.load(ed.i + 1)
        elif k == ord("p"):
            ed.save()
            ed.load(max(0, ed.i - 1))
        elif k == ord("s"):
            ed.save()
        elif k != 255:
            ed.key(k)
    cv2.destroyAllWindows()
    done = sum(bool(e.get("checked")) for e in entries)
    tot = {n: sum(e.get("edits", {}).get(n, 0) for e in entries if e.get("checked"))
           for n in ("deleted", "added", "class_changed", "resized")}
    print(f"다 본 장 {done}/{len(entries)}, 고친 수(다 본 장) {tot}")


if __name__ == "__main__":
    main()
