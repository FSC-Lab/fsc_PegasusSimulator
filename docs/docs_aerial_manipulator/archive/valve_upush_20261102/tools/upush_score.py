#!/usr/bin/env python3
"""(2026-11-02, Pegasus archive/valve_upush_20261102) Score the U-push runs: the isolation's row per run
(outcome, beam turn, arm, tilt, contact force -- through ../../valve_cup_abort_20261102/tools/cup_score.py),
plus, over the turn (push:start..push:end):
  FOLLOW  the beam's turn against the planned claw angle: the valve's lag behind the plan (deg), the claw's
          lead over the fin along its circle (mm), the claw's radial offset from the fin's radius (mm, + out)
  PARTS   which links touched the valve, where (the rotor frame: x radial, y along the turn, z height above the
          rotor origin = the plate bottom), and the mean force on the valve -- from runs/<tag>.contacts.csv

    PYTHONNOUSERSITE=1 /usr/bin/python3 -I upush_score.py [tag ...]
"""
import glob, importlib.util, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "..", "sim", "runs") + "/"
_spec = importlib.util.spec_from_file_location(
    "cupscore", os.path.join(HERE, "..", "..", "valve_cup_abort_20261102", "tools", "cup_score.py"))
cs = importlib.util.module_from_spec(_spec)
_argv, sys.argv = sys.argv, sys.argv[:1]
_spec.loader.exec_module(cs)
sys.argv = _argv
cs.RUNS = RUNS; cs.ss.RUNS = RUNS; cs.ss.s.RUNS = RUNS; cs.ss.s.cs.R = RUNS
_cp = importlib.util.spec_from_file_location("contactparts", os.path.join(HERE, "contact_parts.py"))
AX = np.array([-0.0935, -0.0014, -0.0899])      # obj_0 -> the bearing axis (push_pull_valve_axis_offset)
R_FIN = 0.160

def qrot(w, x, y, z):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def follow_lines(tag):
    d = np.load(RUNS + tag + ".npz", allow_pickle=True)
    t0, t1 = cs.mark(d, "push:start"), cs.mark(d, "push:end")
    t_ab = cs.event_t(d, "push_pull: ABORTING")
    if t0 is None:
        return ["FOLLOW no turn"]
    if t1 is None or (t_ab is not None and t_ab < t1):
        t1 = t_ab if t_ab is not None else d["box"][-1, 0]
    b = d["box"]; k0 = np.searchsorted(b[:, 0], t0)
    w, x, y, z = b[k0, 4:8]
    axis = b[k0, 1:4] + qrot(w, x, y, z) @ AX
    yaw_b = np.unwrap(cs.yaw_of(b[:, 4:8]))
    ref = d["ref"]; cl = d["claw"]
    tt = ref[(ref[:, 0] >= t0) & (ref[:, 0] <= t1), 0]
    if not len(tt):
        return ["FOLLOW no reference in the turn"]
    def ang(P):
        return np.unwrap(np.arctan2(P[:, 1] - axis[1], P[:, 0] - axis[0]))
    a_ref = ang(ref[:, 1:4]); a_cl = ang(cl[:, 1:4])
    th_ref = np.interp(tt, ref[:, 0], a_ref) - np.interp(t0, ref[:, 0], a_ref)
    th_b = np.interp(tt, b[:, 0], yaw_b) - np.interp(t0, b[:, 0], yaw_b)
    th_c = np.interp(tt, cl[:, 0], a_cl) - np.interp(t0, cl[:, 0], a_cl)
    r_c = np.interp(tt, cl[:, 0], np.hypot(cl[:, 1] - axis[0], cl[:, 2] - axis[1]))
    lag = np.degrees(th_ref - th_b); lead = (th_c - th_b) * R_FIN * 1e3
    return [f"FOLLOW beam {np.degrees(th_b[-1]):+6.1f} of planned {np.degrees(th_ref[-1]):+6.1f} deg | valve lag "
            f"max {lag.max():5.1f} end {lag[-1]:5.1f} deg | claw ahead of the fin {lead.min():+5.0f}..{lead.max():+5.0f} mm "
            f"(end {lead[-1]:+4.0f}) | claw radius - fin {(r_c.min()-R_FIN)*1e3:+4.0f}..{(r_c.max()-R_FIN)*1e3:+4.0f} mm"]

def parts_lines(tag):
    import io, contextlib
    cp = importlib.util.module_from_spec(_cp)
    buf = io.StringIO()
    _a, sys.argv = sys.argv, ["contact_parts.py", RUNS, tag]
    try:
        if not os.path.exists(RUNS + tag + ".contacts.csv"):
            return ["PARTS no contact log"]
        with contextlib.redirect_stdout(buf):
            _cp.loader.exec_module(cp)
    finally:
        sys.argv = _a
    return ["PARTS " + l.strip() for l in buf.getvalue().splitlines() if l.startswith("  ") or "offset" in l]

def main(tags):
    for tag in tags:
        print("=" * 120)
        try:
            cs.main([tag])
        except Exception as exc:
            print(f"{tag}: cup_score failed: {exc!r}")
        for l in follow_lines(tag) + parts_lines(tag):
            print("                " + l)

if __name__ == "__main__":
    main(sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(RUNS + "*.npz")))
