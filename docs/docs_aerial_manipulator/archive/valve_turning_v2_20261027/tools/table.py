#!/usr/bin/env python3
"""The 2026-10-27 v2-valve table: config A (the 10-22 final) and B (+ the box push's hand-over
settle gate) on the v2 asset at the measured 0.072 N.m stick-slip friction, beside the 10-26
heading sweep's two 90 deg / 12 s references (v1 asset, 0.048 N.m drive, nominal). Columns: outcome, beam turned, how far the rotor
turned under the open jaws before the close, where along the 40 mm fin the jaws closed, Ready ->
close time, q3 min, % of the turn on a joint stop, peak tilt (Isaac truth)."""
import importlib.util, os, re, subprocess, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); A = os.path.join(HERE, "..", "..")
spec = importlib.util.spec_from_file_location("g", os.path.join(A, "valve_turning_safety_20261022/tools/grasp_offset.py"))
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
RUNS = [("runs", f"t90{c}_{k}", c) for k in ("nom_1", "nom_2", "hw_2", "hw_3", "hw_4") for c in "ABW"]
REF = [("../valve_turning_20261015/runs", t, "ref") for t in ("yw_ref_f048", "yw_ref_f048_2")]
def score(tags, ref):
    out = subprocess.run([sys.executable, "-I", os.path.join(HERE, "score.py")] + (["--ref"] if ref else []) + tags,
                         capture_output=True, text=True, env=dict(os.environ, PYTHONNOUSERSITE="1")).stdout
    return {l.split()[0]: l for l in out.splitlines() if l and not l.startswith(" ")}
have = [r for r in RUNS if os.path.exists(os.path.join(HERE, "..", r[0], r[1] + ".npz"))]
rows = {**score([t for _, t, _ in have], False), **score([t for _, t, _ in REF], True)}
print(f"{'run':16s} {'cfg':4s} {'drift':5s} {'outcome':12s} {'beam':>5s} {'pushed':>7s} {'along':>6s} {'close':>6s} {'q3 min':>7s} {'stop':>5s} {'tilt':>5s}")
for root, t, c in have + REF:
    r = rows.get(t, ""); p = os.path.join(HERE, "..", root, t + ".npz")
    gr = g.grasp(p)
    d = np.load(p, allow_pickle=True); m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    tc = m.get("gripper_close:start", np.nan) - m.get("ready:start", np.nan)
    drift = re.search(r"_hw_(\d)", t); drift = f"s{drift.group(1)}" if drift else "--"
    oc = r[len(t):].split("beam")[0].strip() if r else "?"
    beam = re.search(r"beam\s+([+-]?\d+)", r); q3 = re.search(r"q3\s+(-?[\d.]+)\.\.", r)
    st = re.search(r"stop\s+([\d.]+)%", r); tl = re.search(r"\| tilt\s+([\d.]+)", r)
    pushed = f"{gr[2]:6.1f}" if gr else "   -- "; along = f"{gr[0]:5.1f}" if gr else "  -- "
    if "(no turn)" in r: oc = "no close"
    if "ready:start" not in m: oc = "no Ready"
    print(f"{t:16s} {c:4s} {drift:5s} {oc:12s} {beam.group(1) if beam else '--':>5s} {pushed:>7s} {along:>6s} "
          f"{tc:6.1f} {q3.group(1) if q3 else '--':>7s} {st.group(1) if st else '--':>5s} {tl.group(1) if tl else '--':>5s}")
