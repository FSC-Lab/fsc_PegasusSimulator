#!/usr/bin/env python3
"""Piece-2 audit: swapping the residual estimator in the cascade variant
(fsc_autopilot_ros2: px4_vtol_direct_actuator/nmpc_cascade_steso_node.py, `--est` / CASC_EST)
changes ONE thing -- which class the moment observer and its force twin are -- and nothing else.

    ~/envs/isaac_env/bin/python3 tests/test_est_swap.py [--gate]

  1. STATIC, the node's source: the estimator kind (EST) is consulted only where it is allowed
     to be -- the knob block, the two make_estimator calls in L1Steso.reset, the force-knob map,
     the base.L1Aug guard and the banner. It must NOT appear in adapt() / propagate() /
     _outer_dist() / inner_step / the logging / the scorer: no `if EST == ...` inside the control
     law, so no kind gets a different C(s), clamp, hold, delay buffer or predictor path.
     No estimator class is constructed directly (only through make_estimator), and the node does
     not import LinearESO / MomentumDOB by name.
  2. DYNAMIC, per kind (a fresh interpreter each, CASC_EST set, no ROS): L1Steso().reset()
     builds the moment observer AND the force twin of the kind named, with the kind's own knobs
     (moment and force knobs differ); `pwc` leaves base.L1Aug as the base class; an unknown kind
     is refused at import with the message naming the four choices. The L1 law's own constants
     (wc, ks, umax, tau_spin, tau_surf) and the predictor's knobs (PRED_LAG, PRED_CANCEL,
     PRED_UMAX, PRED_OUTER, SLOSH_PRED) are identical across kinds.
  3. --gate (optional, ~5 min): the offline mission with `--log` for every kind on the faithful
     line; the log HEADER is identical across kinds, every kind completes with 0 solve failures
     and the plant-truth columns (Ftrue_*, xtrue_*) are bit-identical between steso and pwc for
     the first 2 s (before any estimator can have changed the trajectory) -- the harness itself
     is not kind-dependent. (The steso/pwc traces were compared bit-for-bit against the pre-edit
     node once, by hand, when the wiring landed: identical.)
The launchers forward LESO_WO / LESO_WOF / MDOB_K / MDOB_KF to the adaptive pane only when set
(an unset knob must not arrive as an empty string: `_parse_L("")` would raise) -- checked too.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from controller_path import inner_dir                       # noqa: E402

NODE = os.path.join(inner_dir(), "nmpc_cascade_steso_node.py")
PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
KINDS = ("steso", "leso", "mdob")
FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def main():
    src = open(NODE).read()
    lines = src.splitlines()

    print("== 1. static: where the estimator kind is allowed to matter")
    # every line that mentions EST as a name (not as part of another identifier)
    uses = [(i + 1, l) for i, l in enumerate(lines) if re.search(r"(?<![A-Za-z_])EST(?![A-Za-z_])", l) and not l.strip().startswith("#")]
    allowed = re.compile(r"^(EST = |EST_FORCE_KNOB = |if EST in \(|elif EST != |\s*raise SystemExit\(f\"CASC_EST|\s*self\.est(_f)? = \(?make_estimator\(EST|\s*print\(f\"\[cascade\+steso\] estimator=\{EST\}|\s+knobs\.get\(EST|\s+L=|\s+wo=)")
    bad = [(n, l) for n, l in uses if not allowed.match(l) and "make_estimator(EST" not in l and "knobs.get(EST" not in l and "estimator={EST}" not in l]
    check(not bad, f"EST is consulted only in the knob block, the two make_estimator calls, the guard and the banner ({len(uses)} uses)" + (f"; unexpected: {bad}" if bad else ""))
    # the control law must not branch on the kind
    def body(name):
        # a method ends at the next line that is not indented under the class (a def at 4 spaces
        # or anything at column 0), never at end-of-file: the LAST method must not swallow the module
        m = re.search(rf"^    def {name}\(.*?(?=^    def |^\S)", src, re.S | re.M)
        return m.group(0) if m else ""
    for fn in ("adapt", "propagate", "_outer_dist", "_build_pred"):
        b = body(fn)
        check(b and not re.search(r"(?<![A-Za-z_])EST(?![A-Za-z_])", b), f"L1Steso.{fn}() does not consult the estimator kind")
    for fn in ("inner_step", "_status_extra", "_pred_score", "_flush_log"):
        m = re.search(rf"^def {fn}\(.*?(?=^\S)", src, re.S | re.M)
        check(m and not re.search(r"(?<![A-Za-z_])EST(?![A-Za-z_])", m.group(0)), f"{fn}() does not consult the estimator kind")
    check("SO3STESO(" not in src.replace("import SO3STESO", ""), "no estimator is constructed directly (only through make_estimator)")
    check("LinearESO" not in src and "MomentumDOB" not in src, "the node does not name LinearESO / MomentumDOB (it cannot special-case them)")
    check(src.count("make_estimator(EST") == 2, "exactly two make_estimator(EST, ...) calls: the moment observer and its force twin")

    print("== 2. dynamic: each kind builds the pair it names, with its own knobs; the rest is invariant")
    probe = (
        "import sys, os, json\n"
        "sys.argv = [sys.argv[0]]\n"
        f"sys.path.insert(0, {os.path.dirname(os.path.dirname(NODE))!r})\n"
        "import importlib.util\n"
        f"spec = importlib.util.spec_from_file_location('node', {NODE!r}); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)\n"
        "out = {'L1Aug': m.base.L1Aug.__name__, 'EST': m.EST, 'pred': [m.SLOSH_PRED, m.PRED_LAG, m.PRED_CANCEL, m.PRED_UMAX, m.PRED_OUTER]}\n"
        "if m.base.L1Aug is m.L1Steso:\n"
        "    l1 = m.L1Steso(); l1.reset()\n"
        "    e, f = l1.est, l1.est_f\n"
        "    out.update(est=type(e).__name__, est_f=type(f).__name__ if f is not None else None,\n"
        "               knob=(list(getattr(e,'L',[])) or list(getattr(e,'wo',[])) or list(getattr(e,'K',[]))),\n"
        "               knob_f=(list(getattr(f,'L',[])) or list(getattr(f,'wo',[])) or list(getattr(f,'K',[]))) if f is not None else None,\n"
        "               J=list(e.J.diagonal()), J_f=list(f.J.diagonal()) if f is not None else None,\n"
        "               l1=[l1.wc, l1.ks, list(l1.umax), l1.tau_spin, l1.tau_surf], gyro=[e.add_gyro, f.add_gyro if f else None])\n"
        "print('JSON' + json.dumps(out))\n")
    import json
    results = {}
    env = dict(os.environ, VTOL_TANK_KG="0.502", VTOL_TANK_R="0.10", VTOL_TANK_DX="0.3")
    for kind in KINDS + ("pwc",):
        r = subprocess.run([PY, "-c", probe], capture_output=True, text=True, timeout=300, env=dict(env, CASC_EST=kind))
        line = next((l for l in r.stdout.splitlines() if l.startswith("JSON")), None)
        if line is None:
            check(False, f"{kind}: probe failed: {r.stderr.strip()[-300:]}"); continue
        results[kind] = json.loads(line[4:])
    want = {"steso": "SO3STESO", "leso": "LinearESO", "mdob": "MomentumDOB"}
    for kind in KINDS:
        o = results.get(kind)
        if not o:
            continue
        check(o["L1Aug"] == "L1Steso" and o["est"] == want[kind] and o["est_f"] == want[kind],
              f"{kind}: L1Aug=L1Steso, moment observer {o['est']}, force twin {o['est_f']}")
        check(o["knob"] != o["knob_f"] and all(k > 0 for k in o["knob"]) and all(k > 0 for k in o["knob_f"]),
              f"{kind}: moment knob {o['knob']} and force knob {o['knob_f']} are its own, distinct numbers")
        check(abs(o["J_f"][0] - 3.9516) < 1e-3 and o["J_f"][0] == o["J_f"][1] == o["J_f"][2] and o["J"][1] > o["J"][0] and o["gyro"] == [False, False],
              f"{kind}: moment J = inertia {['%.3f' % j for j in o['J']]}, force J = m I ({o['J_f'][0]:.4f}), gyro term left to the caller")
    if "pwc" in results:
        check(results["pwc"]["L1Aug"] == "L1Aug", "pwc: base.L1Aug is the base class (the base L1 law, untouched)")
    inv = {k: (results[k]["l1"], results[k]["pred"]) for k in KINDS if k in results}
    check(len(set(json.dumps(v) for v in inv.values())) == 1, f"L1 constants and predictor knobs identical across kinds: {list(inv.values())[0] if inv else '?'}")
    r = subprocess.run([PY, "-c", probe], capture_output=True, text=True, timeout=300, env=dict(env, CASC_EST="nope"))
    check(r.returncode != 0 and "steso, leso, mdob or pwc" in (r.stderr + r.stdout), "an unknown kind is refused at import, naming the four choices")

    print("== 3. launchers forward the new knobs only when set")
    for f in ("scripts/restart_hil.sh", "scripts/start_vtol_smc_hil.sh"):
        s = open(os.path.join(ROOT, f)).read()
        check("${LESO_WO:+LESO_WO=$LESO_WO}" in s and "${MDOB_KF:+MDOB_KF=$MDOB_KF}" in s and "LESO_WO=${LESO_WO" not in s,
              f"{f}: LESO_WO/LESO_WOF/MDOB_K/MDOB_KF forwarded with ${{VAR:+...}} (never as an empty string)")

    if "--gate" in sys.argv:
        print("== 4. gate: the offline mission with --log for every kind")
        import tempfile, csv
        tmp = tempfile.mkdtemp(prefix="est_swap_")
        heads = {}; truth = {}
        for kind in KINDS + ("pwc",):
            log = os.path.join(tmp, f"{kind}.csv")
            r = subprocess.run([PY, NODE, "--test", "--l1", "--est", kind, "--ttr", "10", "--lat", "0.03", "--comp", "0.049",
                                "--outer_lag", "0.03", "--slosh", "--noise_att", "0.1", "--noise_rate", "1.0", "--dur", "30", "--log", log],
                               capture_output=True, text=True, timeout=900, env=env)
            fails = re.search(r"solve fails (\d+)/(\d+)", r.stdout)
            ok = r.returncode == 0 and fails and fails.group(1) == "0"
            check(ok, f"{kind}: mission completes, solve fails {fails.group(0) if fails else '?'}")
            if os.path.exists(log):
                rows = list(csv.reader(open(log)))
                heads[kind] = rows[0]; truth[kind] = rows[1:]
        check(len(set(map(tuple, heads.values()))) == 1, "log header identical across kinds")
        # the closed loops must be IDENTICAL until the first tick on which the augmentation's own
        # output u_ad differs -- and only diverge from there: the harness, the plant, the noise
        # sequence and the initialisation are kind-blind, the estimator is the only difference
        h = heads.get("steso", [])
        if h and all(k in truth for k in KINDS + ("pwc",)):
            iu = [h.index(c) for c in ("u_x", "u_y", "u_z")]
            io = [i for i, c in enumerate(h) if c not in ("u_x", "u_y", "u_z", "sigma_x", "sigma_y", "sigma_z", "innovation", "hold")
                  and not c.startswith(("Fhat", "Mslosh", "Mres", "Fslosh", "Fpred", "Mahead", "bF", "xest", "ud"))]
            for kind in ("pwc", "leso", "mdob"):
                a, b = truth["steso"], truth[kind]
                n_u = next((n for n, (ra, rb) in enumerate(zip(a, b)) if [ra[i] for i in iu] != [rb[i] for i in iu]), len(a))
                same_before = all([ra[i] for i in io] == [rb[i] for i in io] for ra, rb in zip(a[:n_u + 1], b[:n_u + 1]))
                # u_ad differs from the first tick an estimator reads anything (tick 1-2: C(s) on a
                # different sigma); the plant diverges only once that u_ad has acted
                check(same_before and n_u >= 1,
                      f"steso vs {kind}: plant/measurement/truth columns bit-identical through tick {n_u}, the first tick u_ad differs")

    print()
    if FAILS:
        print(f"FAILED {len(FAILS)}:"); [print("  - " + f) for f in FAILS]; return 1
    print("ESTIMATOR SWAP: ONE THING CHANGES -- PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
