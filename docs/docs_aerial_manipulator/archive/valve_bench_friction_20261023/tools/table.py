#!/usr/bin/env python3
"""The campaign's comparison table: this campaign's runs (bench friction law) beside the paired
10-22 FINAL runs at the 0.048 N.m scene default. Columns: outcome, beam turned, how far the FREE
rotor turned under the open jaws before the close, where along the 40 mm fin the jaws closed,
q3 min, % of the turn on a joint stop, peak tilt (Isaac truth)."""
import importlib.util, os, re, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__)); A = os.path.join(HERE, "..", "..")
spec = importlib.util.spec_from_file_location("g", os.path.join(A, "valve_turning_safety_20261022/tools/grasp_offset.py"))
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
RUNS = [("runs", t, b) for t, b in (("vb_nom_1", "bench"), ("vb_nom_2", "bench"), ("vb_nom_3", "bench"),
        ("vb_hw_2", "bench"), ("vb_hw_3", "bench"), ("vb_hw_4", "bench"), ("vb_low_nom_1", "low"), ("vb_low_hw_3", "low"))]
REF = [("../valve_turning_safety_20261022/runs", t, "0.048") for t in
       ("s22_final_nom_1", "s22_final_nom_2", "s22_fa0_nom_1", "s22_final_hw_2", "s22_final_hw_3", "s22_final_hw_4")]
def score(tags, ref):
    out = subprocess.run([sys.executable, "-I", os.path.join(HERE, "score.py")] + (["--ref"] if ref else []) + tags,
                         capture_output=True, text=True, env=dict(os.environ, PYTHONNOUSERSITE="1")).stdout
    return {l.split()[0]: l for l in out.splitlines() if l and not l.startswith(" ")}
rows = {**score([t for _, t, _ in RUNS], False), **score([t for _, t, _ in REF], True)}
print(f"{'run':16s} {'bearing':8s} {'drift':5s} {'outcome':12s} {'beam':>5s} {'pushed':>7s} {'along':>6s} {'q3 min':>7s} {'stop':>5s} {'tilt':>5s}")
for root, t, b in RUNS + REF:
    r = rows.get(t, "")
    gr = g.grasp(os.path.join(HERE, "..", root, t + ".npz")) if True else None
    drift = re.search(r"_hw_(\d)", t); drift = f"s{drift.group(1)}" if drift else "--"
    oc = r[len(t):].split("beam")[0].strip() if r else "?"
    beam = re.search(r"beam\s+([+-]?\d+)", r); q3 = re.search(r"q3\s+(-?[\d.]+)\.\.", r)
    st = re.search(r"stop\s+([\d.]+)%", r); tl = re.search(r"\| tilt\s+([\d.]+)", r)
    pushed = f"{gr[2]:6.1f}" if gr else "   -- "; along = f"{gr[0]:5.1f}" if gr else "  -- "
    if "(no turn)" in r: oc = "no close"
    print(f"{t:16s} {b:8s} {drift:5s} {oc:12s} {beam.group(1) if beam else '--':>5s} {pushed:>7s} {along:>6s} "
          f"{q3.group(1) if q3 else '--':>7s} {st.group(1) if st else '--':>5s} {tl.group(1) if tl else '--':>5s}")
