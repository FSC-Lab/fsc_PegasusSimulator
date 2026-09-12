#!/usr/bin/env python3
"""Is the reference a TRIM? Plug (xref, uref) into the plant and measure what is left over.

THE METHOD (from the 2026-09-06 reference audit, which found 5.3 rad/s^2 of residual pitch and
established that the reference profiles were not trims at all): a reference the solver is asked to
track must be a state the aircraft can actually hold. Substituting it into the same ODE the
controller predicts with gives the accelerations that reference implies. For a steady point they
should be ~0; for an accelerating one they should equal the acceleration the profile intends, and
nothing else.

WHY NOW: build_refs has since acquired a committed TILT HOLD that overrides the schedule's tilt and
re-solves the thrust split, a speed-gated pitch, an optional nose-down dive, and rate limits on
both the pitch and tilt REFERENCES. Every one of those edits the (xref, uref) package. A trim is a
package -- bags full_tr7 and full_tr25/27 were both lost to shipping one part of it without the
others -- so the whole thing has to be re-checked, not assumed.

Reports, per commanded speed:
    ax    forward acceleration implied   (should match the profile's intent, ~Vc/ttr)
    az    vertical acceleration implied  (should be ~0, or the commanded sink)
    p,q,r body angular accelerations     (should be ~0 -- the reference must not be rotating)
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros2", "px4_vtol_direct_actuator",
                                "px4_vtol_direct_actuator"))

import nmpc_full_smc_node as N          # noqa: E402
import nmpc_so3_transition_node as so3  # noqa: E402


def main():
    Vc = 16.0
    sched = N.Schedule(V_cruise=Vc, tilt_cap_deg=90.0, alpha_max_deg=8.0)
    nmpc = type("stub", (), {"N": 8, "DT": 0.20})()
    F = so3.build_ode()

    print(f"  reference audit: Vc={Vc}, tilt_hold={N.TILT_HOLD_DEG} deg, "
          f"commit={N.TILT_COMMIT_V} m/s, dive={N.DIVE_DEG} deg, sink={N.TRANS_SINK} m/s")
    print("     V   tilt  pitch |    ax     az  |    p      q      r   (rad/s^2)   verdict")
    worst_q = 0.0
    for v in (0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 15.0, 16.0):
        xr, ur, _, _ = N.build_refs(
            sched, nmpc, np.zeros(3), 0.0, 20.0, v, Vc, 40.0, 0.0, False,
            v_wb=N.WINGBORNE_V, sink=N.TRANS_SINK, z_floor=12.0,
            tilt_hold=N.TILT_HOLD_DEG, tilt_hold_v=N.TILT_HOLD_V,
            tilt_commit_v=N.TILT_COMMIT_V, tilt_commit_band=N.TILT_COMMIT_BAND,
            tilt_margin=math.radians(N.TILT_MARGIN_DEG),
            pitch_gate_v=N.PITCH_GATE_V, dive_deg=N.DIVE_DEG)
        x0 = np.asarray(xr[0], float)
        u0 = np.asarray(ur[0], float)
        xdot = np.array(F(x0, u0)).ravel()

        R = x0[3:12].reshape(3, 3)
        tilt = math.degrees(x0[12])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
        # accelerations in the WORLD frame; body-forward is what the profile accelerates along
        acc = xdot[13:16]
        ax = float(acc[0])
        az = float(acc[2])
        p, q, r = (float(xdot[16]), float(xdot[17]), float(xdot[18]))
        worst_q = max(worst_q, abs(q))
        bad = []
        if abs(q) > 0.5:
            bad.append("PITCH RESIDUAL")
        if abs(p) > 0.5 or abs(r) > 0.5:
            bad.append("LATERAL RESIDUAL")
        if abs(az) > 1.5:
            bad.append("VERTICAL RESIDUAL")
        print(f"  {v:5.1f} {tilt:6.1f} {pitch:+6.1f} | {ax:+6.2f} {az:+6.2f} | "
              f"{p:+6.2f} {q:+6.2f} {r:+6.2f}   {'  '.join(bad) if bad else 'ok'}")
    print(f"\n  worst |pitch acceleration| across the corridor: {worst_q:.3f} rad/s^2")

    # ★ THE BACK-TRANSITION IS A REFERENCE TOO, and it has its own trim_split calls -- the
    # nose-up flare and the AFT tilt both re-solve the split. The forward corridor passing says
    # nothing about them, and the decel branch is where the flown departures actually happened.
    print("\n  BACK-TRANSITION (decel: flare + aft tilt)")
    print("     V   tilt  pitch |    ax     az  |    p      q      r   (rad/s^2)   verdict")
    worst_d = 0.0
    for v in (16.0, 14.0, 12.0, 9.0, 6.0, 3.0, 1.0):
        xr, ur, _, _ = N.build_refs(
            sched, nmpc, np.zeros(3), 0.0, 20.0, v, Vc, 40.0, 0.0, False,
            v_wb=N.WINGBORNE_V, sink=N.TRANS_SINK, z_floor=12.0,
            decel=True, decel_rate=1.5, flare_deg=15.0,
            beta_aft=math.radians(20.0),
            tilt_hold=N.TILT_HOLD_DEG, tilt_hold_v=N.TILT_HOLD_V,
            tilt_commit_v=N.TILT_COMMIT_V, tilt_commit_band=N.TILT_COMMIT_BAND,
            tilt_margin=math.radians(N.TILT_MARGIN_DEG),
            pitch_gate_v=N.PITCH_GATE_V, dive_deg=N.DIVE_DEG)
        x0 = np.asarray(xr[0], float); u0 = np.asarray(ur[0], float)
        xdot = np.array(F(x0, u0)).ravel()
        R = x0[3:12].reshape(3, 3)
        tilt = math.degrees(x0[12])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))
        ax, az = float(xdot[13]), float(xdot[15])
        p_, q_, r_ = float(xdot[16]), float(xdot[17]), float(xdot[18])
        worst_d = max(worst_d, abs(q_))
        bad = []
        if abs(q_) > 0.5:
            bad.append("PITCH RESIDUAL")
        if abs(p_) > 0.5 or abs(r_) > 0.5:
            bad.append("LATERAL RESIDUAL")
        if abs(az) > 1.5:
            bad.append("VERTICAL RESIDUAL")
        print(f"  {v:5.1f} {tilt:6.1f} {pitch:+6.1f} | {ax:+6.2f} {az:+6.2f} | "
              f"{p_:+6.2f} {q_:+6.2f} {r_:+6.2f}   {'  '.join(bad) if bad else 'ok'}")
    print(f"\n  worst |pitch acceleration| in the back-transition: {worst_d:.3f} rad/s^2")
    print("  (the 2026-09-06 audit called anything above ~0.05 a broken reference; it found 5.3)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
