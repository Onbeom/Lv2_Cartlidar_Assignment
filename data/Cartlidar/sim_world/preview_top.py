import json, sys, cv2, numpy as np
lay = json.load(open("data/maze_upper_drawing.json"))
d = json.load(open(sys.argv[1]))
S, ox, oy = 200, 2.4, 0.25          # px/m, 원점 이동
W, H = int(4.8 * S), int(3.6 * S)
img = np.full((H, W, 3), 255, np.uint8)
P = lambda x, y: (int((x + ox) * S), int(H - (y + oy) * S))
for pn in lay["panels"]:
    cx, cy = pn["center"]; L = pn["length"] / 2
    a, b = ((cx - L, cy), (cx + L, cy)) if pn["axis"] == "x" else ((cx, cy - L), (cx, cy + L))
    cv2.line(img, P(*a), P(*b), (120, 100, 80), 2)
sb = d["start_box"]; o = sb["outer"] / 2
cv2.rectangle(img, P(sb["center"][0] - o, sb["center"][1] - o), P(sb["center"][0] + o, sb["center"][1] + o), (150, 150, 150), 1)
for sh in d["shelves"]:
    c = sh["center"]; h = sh["size"][0] / 2
    cv2.rectangle(img, P(c[0] - h, c[1] - h), P(c[0] + h, c[1] + h), (90, 90, 90), 2)
    cv2.putText(img, sh["name"][-1], P(c[0] - h, c[1] + h + 0.02), 0, 0.5, (0, 0, 0), 1)
for pk in d["pucks"]:
    col = (40, 40, 220) if pk["color"] == "red" else (40, 160, 40)
    lo, hi = pk["aabb"]
    cv2.rectangle(img, P(lo[0], lo[1]), P(hi[0], hi[1]), col, -1 if pk["location"] != "floor" else 1)
for t in d["place_tags"]:
    cv2.circle(img, P(*t["center"][:2]), 6, (200, 0, 200), 2)
for b in d["barcodes"]:
    cv2.drawMarker(img, P(*b["center"][:2]), (0, 140, 255), cv2.MARKER_SQUARE, 12, 2)
lt = d["signal_light"]
cv2.circle(img, P(lt["x"], lt["y"]), 7, (0, 200, 255), -1)
cv2.drawMarker(img, P(d["robot"]["x"], d["robot"]["y"]), (255, 0, 0), cv2.MARKER_CROSS, 20, 2)
cv2.putText(img, f"seq {lt['sequence']} R{lt['red']} G{lt['green']} Y{lt['yellow']} ({lt['yellow_shape']}) light {d['light_level']}", (10, 25), 0, 0.6, (0, 0, 0), 1)
cv2.putText(img, "filled=shelf puck, outline=floor puck, magenta=place tag, orange=barcode, yellow dot=signal", (10, 50), 0, 0.5, (0, 0, 0), 1)
cv2.imwrite(sys.argv[2], img)
