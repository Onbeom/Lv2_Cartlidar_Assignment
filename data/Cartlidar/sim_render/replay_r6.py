"""R6 주사위 확인 재생 · 판정: render_r6.py 결과(render_r6.json)를 한 장씩 팔 카메라로 넣고(READ_TAG), 그 장의 /perception/pucks 가
온 뒤 /perception/check_dice 를 부른다. 칸 좌표는 월드 = odom 기준(header.frame_id "odom"), tolerance 0 = 노드 설정(dice.tolerance_m).
판정(인지 판정 기준 v1 R6): 장면마다 통과 · 불통과, 틀린 칸 목록, 칸별 이유(error[9])가 정답과 모두 같으면 일치. 일치 100 %.
결과: <out>/r6_rows.jsonl, <out>/summary.json

실행 (PC, run_replay.sh 가 노드를 띄운 뒤 부른다):
  REPLAY=replay_r6.py Cartlidar/sim_render/run_replay.sh <URDF> <결과> <setup.bash> -- --render <render_r6 결과>
"""
import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point

from robot_interfaces.srv import CheckDice, SetPerceptionMode

from replay_r4 import PuckReplayer
from replay_ros import FPS, to_time

ERR_NAME = {0: "OK", 1: "MISSING", 2: "UNEXPECTED", 3: "MULTIPLE", 4: "OFFSET", 5: "WRONG_COLOR", 6: "WRONG_SHAPE"}


class DiceReplayer(PuckReplayer):
    META = "render_r6.json"

    def __init__(self, a):
        a.mode = "arm"
        super().__init__(a, "dice_replayer_r6")
        self.dice_cli = self.create_client(CheckDice, "/perception/check_dice")

    def check(self, pose, t):
        if not self.dice_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/perception/check_dice 없음")
        req = CheckDice.Request()
        req.header.stamp = to_time(t)
        req.header.frame_id = "odom"
        req.cells = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in pose["cells"]]
        req.expect = [int(e) for e in pose["expect"]]
        req.yellow_shape = int(pose["yellow_shape"])
        req.tolerance = 0.0
        fut = self.dice_cli.call_async(req)
        while not fut.done():
            self.spin_for(0.05)   # 시계는 그대로 (퍽 목록 나이 dice.max_age_s 안)
        return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--render", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--t0", type=float, default=1000.0)
    ap.add_argument("--start-delay", type=float, default=1.0)
    ap.add_argument("--qos-depth", type=int, default=10)
    ap.add_argument("--settle", type=float, default=0.05)
    ap.add_argument("--max-wait", type=float, default=1.0)
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    n = DiceReplayer(a)
    n.set_clock(a.t0)
    n.spin_for(1.0)
    n.set_mode(SetPerceptionMode.Request.READ_TAG)
    rows = []
    t = a.t0 + a.start_delay
    try:
        for pose in n.meta["poses"]:
            if a.only and not any(s in pose["name"] for s in a.only):
                continue
            n.publish(pose, t)
            n.spin_for(a.settle)
            got, tc = None, t
            for _ in range(int(a.max_wait * FPS)):
                tc += 1.0 / FPS
                n.set_clock(tc)
                n.spin_for(a.settle)
                got = next((m for m in reversed(n.puck_msgs) if abs(m["stamp"] - t) < 1e-6), None)
                if got:
                    break
            res = n.check(pose, t) if got else None
            err = list(res.error) if res else None
            match = bool(res and res.observed and res.passed == pose["expected_passed"] and
                         sorted(res.wrong) == pose["expected_wrong"] and err == pose["expected_error"])
            seen = sorted(p["tag_id"] for p in got["pucks"]) if got else None
            rows.append({"pose": pose["name"], "kind": pose["kind"], "die": pose["die"], "pips": pose["pips"],
                         "light_level": pose["light_level"], "answered": got is not None,
                         "observed": bool(res and res.observed), "message": res.message if res else None,
                         "passed": res.passed if res else None, "expected_passed": pose["expected_passed"],
                         "wrong": list(res.wrong) if res else None, "expected_wrong": pose["expected_wrong"],
                         "error": [ERR_NAME[e] for e in err] if err else None,
                         "expected_error": [ERR_NAME[e] for e in pose["expected_error"]],
                         "offset": [round(float(o), 4) for o in res.offset] if res else None,
                         "placed_tags": sorted(p["tag_id"] for p in pose["pucks"]), "seen_tags": seen,
                         "pucks": got["pucks"] if got else None, "match": match})
            r = rows[-1]
            print(r["pose"], "일치" if match else "불일치", "틀린 칸", r["wrong"], "기대", r["expected_wrong"],
                  "" if match else f"이유 {r['error']} 읽은 번호 {seen} 놓은 번호 {r['placed_tags']}", flush=True)
            t = tc + 1.0 / FPS
    finally:
        try:
            n.set_mode(SetPerceptionMode.Request.OFF)
        except Exception as e:  # noqa: BLE001
            print("set_mode OFF 실패:", e)
    summary = {"mode": "r6", "frames": len(rows), "match": sum(r["match"] for r in rows),
               "rate": round(sum(r["match"] for r in rows) / len(rows), 4) if rows else None,
               "by_kind": {k: f"{sum(r['match'] for r in rows if r['kind'] == k)}/{sum(1 for r in rows if r['kind'] == k)}"
                           for k in dict.fromkeys(r["kind"] for r in rows)},
               "by_light": {f"{lv:g}": f"{sum(r['match'] for r in rows if r['light_level'] == lv)}/"
                                       f"{sum(1 for r in rows if r['light_level'] == lv)}"
                            for lv in sorted({r['light_level'] for r in rows})},
               "mismatches": [r["pose"] for r in rows if not r["match"]],
               "max_ok_offset_m": max((o for r in rows if r["offset"] for i, o in enumerate(r["offset"])
                                       if r["expected_error"][i] == "OK" and r["pucks"] is not None and o > 0), default=None),
               "render": str(a.render)}
    summary["pass"] = bool(rows and summary["match"] == len(rows))
    with open(a.out / "r6_rows.jsonl", "w") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
