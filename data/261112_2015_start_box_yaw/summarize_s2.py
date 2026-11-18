"""run_s2_all.sh 결과 폴더의 경우마다 check_s2.py 를 돌려 한 줄씩 (판정 · 회전 집기마다 n · r)."""
import json
import os
import subprocess
import sys

R = sys.argv[1]
H = os.path.dirname(os.path.abspath(__file__))
for c in ["b0", "b15", "bm15", "b30", "bm30", "b45", "bm45", "s0", "sm30"]:
    d = json.loads(subprocess.check_output([sys.executable, os.path.join(H, "check_s2.py"), os.path.join(R, c)]))
    turns = [f"{r['key']}{'L' if r['puck'] == '-1' else 'R'} {r['plan']}->{r['sent']} r {r['r_real']:.2f} (격자 {r['n_grid']:+.2f}/{r['r_grid']:.2f})"
             for r in d["rows"] if "sent" in r]
    print(f"{c}: {d['final']} look {d['looks']} max {d['max_look_err_mm']} mm, fails {d['fails']}")
    for t in turns:
        print("   ", t)
