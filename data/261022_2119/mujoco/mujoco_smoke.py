"""URDF/MJCF load-and-step test only; not a locomotion or controller test.

출처: velog 교안 "2026-10-22 URDF,USD -> Mujoco" 9절 code/mujoco_smoke.py
(https://velog.io/@roboticsmaster/2026-10-22-URDFUSD-Mujoco) 내용을 그대로 옮김. 바꾼 것 없음.
"""
import argparse
import csv
import json
from pathlib import Path
import mujoco
import numpy as np

def run(source, output, steps=500):
    if steps <= 0:
        raise ValueError("steps must be positive")
    source, output = Path(source).resolve(), Path(output).resolve()
    model = mujoco.MjModel.from_xml_path(str(source))
    if model.nv == 0:
        raise ValueError("No moving DOFs; inspect the imported joint structure")
    # Isolate load/step behavior; this is not a gravity/contact test.
    model.opt.gravity[:] = 0
    data = mujoco.MjData(model)
    output.mkdir(parents=True, exist_ok=False)
    mujoco.mj_saveLastXML(str(output / "compiled.xml"), model)
    rows = 0
    with (output / "state.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["step", "time_s", "max_abs_qvel", "finite"])
        for step in range(steps):
            mujoco.mj_step(model, data)
            finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
            writer.writerow([step, data.time, float(np.max(np.abs(data.qvel))), finite])
            rows += 1
            if not finite:
                raise RuntimeError("Non-finite state; inspect CSV and model")
    report = {"status": "PASS_LOAD_STEP_ONLY", "source": str(source),
              "mujoco_version": mujoco.__version__, "steps": rows,
              "nq": model.nq, "nv": model.nv, "nu": model.nu,
              "gravity": [0, 0, 0], "command": "none",
              "not_tested": ["locomotion", "gravity", "ground contact", "controller"]}
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--steps", type=int, default=500)
    args = parser.parse_args()
    run(args.source, args.output, args.steps)
