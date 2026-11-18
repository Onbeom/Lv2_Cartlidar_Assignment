#!/usr/bin/env python3
"""3D 라벨 고치기(사람용). 자동 초안을 한 퍽씩 크게 보며 맞으면 넘기고, 틀리면 모서리를 끌어 고친다.

실행 (PC): ~/yolo-venv/bin/python fix_pose.py --labels labels_train3.json
  순서: 고칠 가능성이 큰 것(check → no_tag → fail)부터. --all 이면 auto 도 함께, --all-again 이면 이미 본 것도.
  pick_review.py 로 고른 줄(review)이 있으면 그것만 보여 준다(--all 이면 전부).

마우스: 왼쪽 끌기 = 동그라미(모서리 · 원기둥은 위아래 끝면 가운데)를 옮겨 고정(노란색이 됨), 놓으면 나머지가 따라 맞음
        오른쪽 클릭 = 그 동그라미 고정 풀기
키:     스페이스 / n = 이 라벨 맞음, 다음 · p = 앞 퍽 · x = 라벨 불가(가려짐 · 안 보임), 다음
        f = 뒤집힌 자세 후보로 바꾸기(긴 축이 반대로 기운 경우) · r = 처음 초안으로
        g = 원기둥 ↔ 직육면체 · h = 빨강 ↔ 초록 (YOLO 종류가 틀렸을 때, 새 모양으로 초안을 다시 맞춤)
        t = 점 찍기 ↔ 끌기. 점 찍기(직육면체 기본): 실제 퍽의 보이는 모서리를 순서 없이 왼쪽 클릭(빨간 +),
            3개 이상이면 자세를 처음부터 다시 찾음 · 오른쪽 클릭 = 가까운 점 지우기
        w a s d = 위치 2 px 이동 · z c = 멀리 · 가까이 · q e = 긴 축 둘레 회전(직육면체) · i k j l = 긴 축 기울이기
        v = 선 숨기기 · 스페이스 전까지 저장 안 함 · Esc = 끝(저장됨)
"""
import argparse, json, os
from pathlib import Path
import cv2, numpy as np
import puck3d as P, fit_pose as F
ROOT = Path('/home/pa4/git/lv2_next_module')
ORDER = {'check': 0, 'no_tag': 1, 'fail': 2, 'auto': 3}
VIEW = 820
EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count('1') == 1]

def handles(cls):
    """(몸체 점 N×3). 직육면체 = 코너 8개, 원기둥 = 끝면 가운데 2개."""
    return np.array([[0, 0, P.HL], [0, 0, -P.HL]]) if P.is_cyl(cls) else P.CORNERS

class Editor:
    def __init__(self, path, statuses, again=False):
        self.path = Path(path); self.J = json.load(open(path)); self.K = np.array(self.J['K'], float); self.D = None if self.J['D'] is None else np.array(self.J['D'], float)
        self.setdir = {'train3': ROOT / '261104_1618_E3/train3_arm', 'final': ROOT / '261105_2047_final_eval/final_arm'}[self.J['set']]
        R = self.J['rows']; rv = 'auto' not in statuses and any(r.get('review') for r in R)   # pick_review.py 로 고른 것이 있으면 그것만
        idx = [i for i, r in enumerate(R) if (r.get('review') if rv else r['status'] in statuses) and (again or 'human' not in r)]
        self.order = sorted(idx, key=lambda i: (ORDER[R[i]['status']], R[i]['image'], R[i]['box_index'])); self.k = 0; self.hide = False; self.drag = None; self.load()
    def load(self):
        self.r = self.J['rows'][self.order[self.k]]; self.img = cv2.imread(str(self.setdir / 'images' / self.r['image'])); self.cls = self.r['cls']; self.box = np.array(self.r['box']); self.H_ = handles(self.cls)
        qs = [np.array(q, float) for q in self.r['tags_2d']]; self.quads = qs
        self.hull = F.color_hull(self.img, self.box, self.cls, qs); self.ob = F.Obs(self.cls, self.K, self.D, self.hull); self.ob.free_off = bool(self.J.get('tag_off'))
        hu = self.r.get('human') or {}
        self.pins = {int(k): np.array(v, float) for k, v in hu.get('pins', {}).items()}
        self.clicks = [np.array(u, float) for u in hu.get('clicks', [])]; self.click_mode = not P.is_cyl(self.cls)   # 직육면체는 점 찍기가 기본
        self.click_pins = {int(k): np.array(v, float) for k, v in hu.get('click_pins', {}).items()}
        if 'R' in self.r: self.Rp, self.c = np.array(self.r['R'], float), np.array(self.r['c'], float)
        else:
            s = max(self.box[2] - self.box[0], self.box[3] - self.box[1]); z = self.K[0, 0] * .06 / s; m = (self.box[:2] + self.box[2:]) / 2
            self.Rp, self.c = np.eye(3), np.array([(m[0] - self.K[0, 2]) / self.K[0, 0] * z, (m[1] - self.K[1, 2]) / self.K[1, 1] * z, z])
        s = max(self.box[2] - self.box[0], self.box[3] - self.box[1], 40) * 1.7; m = (self.box[:2] + self.box[2:]) / 2
        self.org = m - s / 2; self.sc = VIEW / s
    def uv(self, X, R=None, c=None): return P.project(X, self.Rp if R is None else R, self.c if c is None else c, self.K, self.D)
    def to_view(self, q): return (np.asarray(q) - self.org) * self.sc
    def from_view(self, p): return np.asarray(p, float) / self.sc + self.org
    def refit(self, tilt=None):
        if self.quads: F.assign_tags_all(self.ob, self.Rp, self.c, self.quads)
        else: self.ob.tags = []
        self.ob.pins = [(self.H_[i], u) for i, u in self.pins.items()] + [(P.CORNERS[i], u) for i, u in getattr(self, 'click_pins', {}).items()]
        self.Rp, self.c, _ = self.ob.fit(self.Rp, self.c)
    def render(self):
        M = np.array([[self.sc, 0, -self.org[0] * self.sc], [0, self.sc, -self.org[1] * self.sc]], np.float32)
        V = cv2.warpAffine(self.img, M, (VIEW, VIEW), flags=cv2.INTER_CUBIC, borderValue=(40, 40, 40))
        if not self.hide:
            b = self.to_view(self.box.reshape(2, 2)).astype(int); cv2.rectangle(V, tuple(b[0]), tuple(b[1]), (120, 120, 120), 1)
            if self.hull is not None: cv2.polylines(V, [self.to_view(self.hull).astype(np.int32)], True, (0, 200, 255), 1)
            for q in self.quads: cv2.polylines(V, [self.to_view(q).astype(np.int32)], True, (255, 0, 255), 1)
            C = self.to_view(self.uv(P.CORNERS))
            if P.is_cyl(self.cls):
                for z, col in ((P.HL, (255, 255, 0)), (-P.HL, (255, 255, 255))):
                    a = np.linspace(0, 2 * np.pi, 48); ring = np.c_[P.HW * np.cos(a), P.HW * np.sin(a), np.full(48, z)]
                    cv2.polylines(V, [self.to_view(self.uv(ring)).astype(np.int32)], True, col, 1, cv2.LINE_AA)
                pp = self.to_view(self.uv(P.CYL)); h = cv2.convexHull(pp.astype(np.float32)).astype(np.int32); cv2.polylines(V, [h], True, (0, 255, 0), 1, cv2.LINE_AA)
            else:
                for a, e in EDGES:
                    col = (255, 255, 0) if P.CORNERS[a, 2] > 0 and P.CORNERS[e, 2] > 0 else (255, 255, 255)
                    cv2.line(V, tuple(C[a].astype(int)), tuple(C[e].astype(int)), col, 1, cv2.LINE_AA)
            if not self.click_mode:
                for i, q in enumerate(self.to_view(self.uv(self.H_))):
                    cv2.circle(V, tuple(q.astype(int)), 6, (0, 255, 255) if i in self.pins else (0, 255, 0), 2, cv2.LINE_AA)
            for u in self.clicks:
                q = tuple(self.to_view(u).astype(int)); cv2.drawMarker(V, q, (0, 0, 255), cv2.MARKER_CROSS, 16, 2, cv2.LINE_AA)
        s = self.ob.scores(self.Rp, self.c); d = np.linalg.norm(self.c) * 100
        txt = [f"{self.k + 1}/{len(self.order)} {self.r['image']} #{self.r['box_index']} {P.NAMES[self.cls]}", f"{self.r['status']} tag {len(self.quads)} sil {s.get('sil_iou', 0):.3f} tagpx {s.get('tag_px', 0):.2f} dist {d:.1f}cm pins {len(self.pins)} clicks {len(self.clicks)}", "mode: " + ("CLICK corners (t: drag)" if self.click_mode else "DRAG handles (t: click)")]
        for i, t in enumerate(txt): cv2.putText(V, t, (8, 22 + 22 * i), 0, .55, (0, 0, 0), 3, cv2.LINE_AA); cv2.putText(V, t, (8, 22 + 22 * i), 0, .55, (255, 255, 255), 1, cv2.LINE_AA)
        return V
    def mouse(self, ev, x, y, fl, _):
        if self.click_mode:
            if ev == cv2.EVENT_LBUTTONDOWN: self.clicks.append(self.from_view((x, y))); self.refit_clicks()
            elif ev == cv2.EVENT_RBUTTONDOWN and self.clicks:
                d = [np.linalg.norm(self.to_view(u) - (x, y)) for u in self.clicks]; i = int(np.argmin(d))
                if d[i] < 20: del self.clicks[i]; self.refit_clicks()
            return
        p = self.to_view(self.uv(self.H_)); d = np.linalg.norm(p - (x, y), axis=1); i = int(d.argmin())
        if ev == cv2.EVENT_LBUTTONDOWN and d[i] < 14: self.drag = i
        elif ev == cv2.EVENT_MOUSEMOVE and self.drag is not None: self.pins[self.drag] = self.from_view((x, y))
        elif ev == cv2.EVENT_LBUTTONUP and self.drag is not None: self.pins[self.drag] = self.from_view((x, y)); self.drag = None; self.refit()
        elif ev == cv2.EVENT_RBUTTONDOWN and d[i] < 14 and i in self.pins: del self.pins[i]; self.refit()
    def refit_clicks(self):
        """찍은 점이 3개 이상이면 자세를 처음부터 다시 찾고, 그보다 적으면 지금 자세에서 조금만 맞춘다."""
        self.click_pins = {}
        if len(self.clicks) >= 3:
            r = F.fit_clicks(self.ob, self.clicks, self.quads, [(self.H_[i], u) for i, u in self.pins.items()])
            if r is not None: self.Rp, self.c, self.click_pins = r; return
            print('찍은 점에 맞는 자세가 없습니다. 점을 지우고(오른쪽 클릭) 다시 찍으세요')
        self.refit()
    def set_cls(self, new):
        """모양 · 색을 바꾸고 그 모양으로 초안을 새로 맞춘다. YOLO 원래 값은 cls_yolo 로 남긴다(YOLO 라벨 파일은 안 바꿈)."""
        if new == self.r['cls']: return
        self.r.setdefault('cls_yolo', self.r['cls']); self.r['cls'] = new; self.r.pop('human', None)
        res = F.fit_puck(self.img, self.box, new, self.quads, self.K, self.D, free_off=bool(self.J.get('tag_off')))
        if res is not None: self.r.update(R=res['R'].tolist(), c=res['c'].tolist())
        else: self.r.pop('R', None); self.r.pop('c', None)
        self.load()
    def nudge(self, t=None, rot=None):
        if t is not None: self.c = self.c + np.asarray(t) * self.c[2] / self.K[0, 0]
        if rot is not None: self.Rp = cv2.Rodrigues(self.Rp @ np.asarray(rot))[0] @ self.Rp
        self.moved = True
    def flip(self):
        v = self.c / np.linalg.norm(self.c); a = self.Rp[:, 2]; b = -(a - 2 * (a @ v) * v); x = np.cross([.3, .5, .8], b); x /= np.linalg.norm(x); self.Rp = np.c_[x, np.cross(b, x), b]; self.refit()
    def save(self, state):
        if getattr(self, 'moved', False):            # 키로 옮겼으면 모든 동그라미를 고정점으로 둔다(내부값이 바뀌어도 2D 로 다시 계산)
            self.pins = {i: u for i, u in enumerate(self.uv(self.H_))}; self.moved = False
        self.r['human'] = dict(state=state, pins={str(i): np.round(u, 2).tolist() for i, u in self.pins.items()}, clicks=[np.round(u, 2).tolist() for u in self.clicks], click_pins={str(i): np.round(u, 2).tolist() for i, u in self.click_pins.items()})
        self.r.update(R=self.Rp.tolist(), c=self.c.tolist(), corners_px=np.round(self.uv(P.CORNERS), 2).tolist(), **self.ob.scores(self.Rp, self.c))
        if self.ob.free_off and self.ob.tags and getattr(self.ob, 'off', None) is not None and len(self.ob.off) == len(self.ob.tags):
            self.r.update(tag_off=self.ob.off.tolist(), tags_body=[np.round(T, 5).tolist() for _, T in self.ob.tags])
        t = self.path.with_suffix('.tmp'); json.dump(self.J, open(t, 'w'), ensure_ascii=False); os.replace(t, self.path)

def main():
    p = argparse.ArgumentParser(); p.add_argument('--labels', required=True); p.add_argument('--all', action='store_true'); p.add_argument('--all-again', action='store_true'); a = p.parse_args()
    st = {'check', 'no_tag', 'fail'} | ({'auto'} if a.all or a.all_again else set()); E = Editor(a.labels, st, a.all_again)
    if not E.order: print('볼 것이 없습니다'); return
    cv2.namedWindow('fix_pose', cv2.WINDOW_AUTOSIZE); cv2.setMouseCallback('fix_pose', E.mouse)
    T, A = 2.0, np.radians(2)
    while True:
        cv2.imshow('fix_pose', E.render()); k = cv2.waitKey(30) & 0xFF
        if k == 255: continue
        if k == 27: break
        if k in (32, ord('n'), ord('x')):
            E.save('skip' if k == ord('x') else 'ok')
            if E.k + 1 >= len(E.order): print('끝까지 봤습니다'); break
            E.k += 1; E.load()
        elif k == ord('p') and E.k > 0: E.k -= 1; E.load()
        elif k == ord('r'): E.pins = {}; E.r.pop('human', None); E.load()
        elif k == ord('f'): E.flip()
        elif k == ord('t'): E.click_mode = not E.click_mode
        elif k == ord('g'): E.set_cls(E.cls ^ 2)        # 원기둥 ↔ 직육면체
        elif k == ord('h'): E.set_cls(E.cls ^ 1)        # 빨강 ↔ 초록
        elif k == ord('v'): E.hide = not E.hide
        elif k == ord('w'): E.nudge(t=(0, -T, 0))
        elif k == ord('s'): E.nudge(t=(0, T, 0))
        elif k == ord('a'): E.nudge(t=(-T, 0, 0))
        elif k == ord('d'): E.nudge(t=(T, 0, 0))
        elif k == ord('z'): E.c = E.c * 1.01; E.moved = True
        elif k == ord('c'): E.c = E.c * .99; E.moved = True
        elif k == ord('q'): E.nudge(rot=(0, 0, A))
        elif k == ord('e'): E.nudge(rot=(0, 0, -A))
        elif k == ord('i'): E.nudge(rot=(A, 0, 0))
        elif k == ord('k'): E.nudge(rot=(-A, 0, 0))
        elif k == ord('j'): E.nudge(rot=(0, A, 0))
        elif k == ord('l'): E.nudge(rot=(0, -A, 0))
    cv2.destroyAllWindows()

if __name__ == '__main__': main()
