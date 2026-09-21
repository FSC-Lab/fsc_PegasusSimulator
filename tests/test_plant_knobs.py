#!/usr/bin/env python3
"""The model-error envelope's plant-side knobs (2026-09-15): each of --plant_mass_scale,
--plant_inertia_scale, --plant_cq, --plant_aero_scale, --plant_thrust_scale, --plant_J0 (and the older
--plant_cg_dx / --tau_spin+--l1_tau_spin) must change the PLANT and nothing the controller flies with.
  1. the neutral values spelled out == the flags absent: the gate's trace is byte-identical;
  2. after a perturbed run the controller's constants (so3.M, its inertia, CQ, aero_body's
     coefficients, the prop mapping's J0/CT0) are exactly what they were -- no leak;
  3. every knob moves the plant: the trace differs from the unperturbed one.
    ~/envs/isaac_env/bin/python3 tests/test_plant_knobs.py      (~2 min)
"""
import hashlib
import os
import subprocess
import sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import controller_dir, load_airframe     # noqa: E402
PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
PKG = os.path.join(controller_dir(), "px4_vtol_direct_actuator")
NODE = os.path.join(PKG, "nmpc_cascade_steso_node.py")
TMP = os.environ.get("TMPDIR", "/tmp")
BASE = ["--test", "--est", "mdob", "--ttr", "10", "--lat", "0.03", "--comp", "0.049", "--outer_lag", "0.03",
        "--slosh", "--noise_att", "0.1", "--noise_rate", "1.0", "--l1", "--dur", "8", "--every", "1.0", "--seed", "3"]
KNOBS = {"mass": ["--plant_mass_scale", "1.1"], "inertia": ["--plant_inertia_scale", "1,1.25,1"],
         "cq": ["--plant_cq", "0.18"], "aero": ["--plant_aero_scale", "pitch=1.3,lift=0.8"],
         "thrust": ["--plant_thrust_scale", "0.85"], "j0": ["--plant_J0", "0.6"],
         "cg": ["--plant_cg_dx", "0.03"], "lag": ["--tau_spin", "0.24", "--l1_tau_spin", "0.12"],
         # the tilt servo (B2, 2026-09-21): the plant's own tau / slew, the controller's model untouched
         "tilt": ["--plant_tilt_tau", "0.20", "--plant_tilt_rate", "50"]}
# the neutral cQ is the AIRFRAME's, read from the file: a literal here was the sim-only 0.12 and
# failed the moment the measured 0.0190 went in (2026-09-21)
_AF = load_airframe()
NEUTRAL = ["--plant_mass_scale", "1.0", "--plant_inertia_scale", "1,1,1", "--plant_cq", repr(float(_AF.rotors.cq)),
           "--plant_thrust_scale", "1.0", "--plant_tilt_tau", repr(float(_AF.tilt.tau)), "--plant_tilt_rate", repr(float(_AF.tilt.rate_deg_s))]
fails = 0


def check(cond, msg):
    global fails
    print(("PASS " if cond else "FAIL ") + msg); fails += 0 if cond else 1


def run(tag, extra):
    log = os.path.join(TMP, f"plant_knob_{tag}.csv")
    cmd = [PY, NODE] + extra + BASE + ["--log", log]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=600, cwd=os.path.dirname(PKG))
    if r.returncode != 0:
        return None, r.stdout[-800:] + r.stderr[-800:]
    return log, r.stdout


def trace_hash(log):
    # the per-tick log without its wall-clock column
    rows = open(log).read().split("\n")
    h = rows[0].split(","); keep = [i for i, c in enumerate(h) if "ms" not in c]
    body = "\n".join(",".join(r.split(",")[i] for i in keep) for r in rows if r)
    return hashlib.md5(body.encode()).hexdigest()


# 1. neutral == absent
l0, out0 = run("plain", []); l1, out1 = run("neutral", NEUTRAL)
check(l0 is not None and l1 is not None, "the gate runs plain and with the neutral knobs")
if l0 and l1:
    check(trace_hash(l0) == trace_hash(l1), "neutral knobs spelled out == flags absent (trace byte-identical)")
    check("PLANT differs" not in out1, "neutral knobs print no PLANT line")
# 2. no leak + 3. the plant moves
LEAK = ("import sys, os; sys.argv=[sys.argv[0]] + %r; "
        "sys.path.insert(0, %r); from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3; "
        "from px4_vtol_direct_actuator import nmpc_cascade_node as base; "
        "before=(so3.M, so3.IXX, so3.IYY, so3.IZZ, so3.CQ, so3.CMA, so3.CLA, so3.CMDE, so3.RPOS.tolist(), base.PROP_J0, base._CT0, so3.TAU_BETA, so3.TILT_RATE); "
        "base.offline_test(); "
        "after=(so3.M, so3.IXX, so3.IYY, so3.IZZ, so3.CQ, so3.CMA, so3.CLA, so3.CMDE, so3.RPOS.tolist(), base.PROP_J0, base._CT0, so3.TAU_BETA, so3.TILT_RATE); "
        "print('LEAK' if before != after else 'NOLEAK', before if before != after else '')")
for tag, extra in KNOBS.items():
    lg, out = run(tag, extra)
    if lg is None:
        check(False, f"{tag}: the gate ran ({out[-200:]!r})"); continue
    check("PLANT" in out or (tag == "lag" and "rotor tau=0.240" in out), f"{tag}: the gate announces the perturbed plant")
    check(trace_hash(lg) != trace_hash(l0), f"{tag}: the trace differs from the unperturbed run (the plant moved)")
    argv = extra + [a for a in BASE if a != "--log"] + ["--dur", "3"]
    r = subprocess.run([PY, "-c", LEAK % (argv, os.path.dirname(PKG))], capture_output=True, text=True, timeout=600, cwd=os.path.dirname(PKG))
    check("NOLEAK" in r.stdout, f"{tag}: the controller's constants are untouched" + ("" if "NOLEAK" in r.stdout else f" ({(r.stdout + r.stderr)[-300:]!r})"))
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
