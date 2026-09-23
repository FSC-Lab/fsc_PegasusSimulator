"""Record what the REFERENCE asks of the inner loop and what that loop actually delivers.

WHY THIS EXISTS.  The cascade's reference is two kinds of object and only one is a property of
the aircraft.  `Schedule.at(V)` -- (Tf, Tr, beta, de_trim, theta_ref) -- is a force and moment
BALANCE of the airframe: the outer NMPC owns it and no inner loop touches it.  The SHAPING (the
flare, the aft-tilt gate, the speed ramp, `hold_k`) was fitted against a loop that does not obey
it exactly, so it carries THAT LOOP'S TRACKING ERROR as part of the plan.  Swap the inner loop and
the same command is delivered differently: on 2026-09-22 the same 14.76 deg flare peak came out at
12.27 deg under the geometric law (83%) and 16.42 under INDI (111%).  This probe is how that was
measured -- it reads theta_ref and theta on the SAME reference, tick by tick, for either loop.

  scripts/ref_probe.py geo|indi <out.csv> -- <gate flags...>
  scripts/ref_metrics.py                      # scores a directory of runs

★ IMPORT THE NODES PACKAGE-QUALIFIED, the way they import each other.  A bare
`import nmpc_cascade_node` with the package dir ALSO on sys.path gives a SECOND module object, and
the variant's `base.MOMENT_LAW = ...` then lands on a copy the gate never reads: the first run of
this probe returned the INDI mission BYTE-IDENTICAL to the geometric one and looked like a null
result.  The assert below is the guard.


Wraps `base.inner_step` (a module-level seam both the ROS node and the gate look up by name),
so the shipped nodes are untouched: every tick's (t, theta_ref, theta, V, beta, elev, tilt) is
captured on the way through. Usage:  refprobe.py geo|indi <out.csv> -- <gate flags...>
"""
import os, sys, math
import numpy as np

kind, out = sys.argv[1], sys.argv[2]
flags = sys.argv[sys.argv.index("--") + 1:]
CTRL = os.path.expanduser("~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator")
# ★ PACKAGE-QUALIFIED, because the nodes import each other that way. A bare
# `import nmpc_cascade_node` with the package dir ALSO on sys.path gives a SECOND module object,
# and the variant's `base.MOMENT_LAW = ...` then lands on a copy the gate never reads: the first
# run of this probe returned the INDI mission BYTE-IDENTICAL to the geometric one.
sys.path.insert(0, os.path.dirname(os.path.dirname(CTRL)))

if kind == "indi":
    os.environ["INDI_MODE"] = "full"
sys.argv = ["gate", "--test"] + flags
if kind == "indi":
    from px4_vtol_direct_actuator import nmpc_cascade_indi_node as M
else:
    from px4_vtol_direct_actuator import nmpc_cascade_imu_node as M
base = M.base
assert (base.MOMENT_LAW is not None) == (kind == "indi"), "the loop under test is not installed"

_inner = base.inner_step
rows = []
_t = [0.0]

def rec(R, w, theta_ref, heading, beta, V, Tf, Tr, de_trim, g, alloc, **kw):
    o = _inner(R, w, theta_ref, heading, beta, V, Tf, Tr, de_trim, g, alloc, **kw)
    th = math.degrees(math.asin(max(-1.0, min(1.0, float(R[2][0] if not hasattr(R, "shape") else R[2, 0])))))
    rows.append((_t[0], math.degrees(theta_ref), th, float(V), math.degrees(beta),
                 float(o[2]), float(w[1]), float(Tf), float(Tr), math.degrees(de_trim)))
    _t[0] += kw.get("dt", 0.005)
    return o

base.inner_step = rec
try:
    base.offline_test()
finally:
    with open(out, "w") as f:
        f.write("t,theta_ref,theta,V,tilt,elev,q,Tf,Tr,de_trim\n")
        for r in rows:
            f.write(",".join("%.6f" % v for v in r) + "\n")
    print("probe: %d ticks -> %s" % (len(rows), out))
