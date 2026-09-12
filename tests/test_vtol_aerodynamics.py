#!/usr/bin/env python3
"""Physical-correctness regression tests for the VTOL aerodynamic model.

These assert properties that ANY wing must have, stated in terms of a
physically-defined flight condition (an attitude plus a world velocity) rather
than in terms of the model's own internal variables.  That is the whole point:
the 2026-09-06 audit found six wrong-signed derivatives precisely because the
model was only ever checked against its own (FLU) definition of alpha, which
is the NEGATIVE of the standard aerodynamic angle of attack.

Run with any python that has numpy + scipy (e.g. the isaac_env venv):

    ~/envs/isaac_env/bin/python3 tests/test_vtol_aerodynamics.py

No Isaac Sim required -- the aero model is pure numpy.
"""

import importlib.util
import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

_HERE = os.path.dirname(os.path.abspath(__file__))
_AERO_PY = os.path.join(
    _HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
    "logic", "dynamics", "vtol_aerodynamics.py")

_spec = importlib.util.spec_from_file_location("vtol_aerodynamics", _AERO_PY)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
VTOLAerodynamics = _mod.VTOLAerodynamics


class FakeState:
    """Duck-typed pegasus State: FLU body velocity + FLU body rates."""

    def __init__(self, linear_body_velocity, angular_velocity=(0.0, 0.0, 0.0)):
        self.linear_body_velocity = np.array(linear_body_velocity, dtype=float)
        self.angular_velocity = np.array(angular_velocity, dtype=float)


def level_flight_state(speed, aoa_deg, omega_flu=(0.0, 0.0, 0.0)):
    """Body state for LEVEL flight along world +x with the body pitched
    nose-UP by `aoa_deg` -- i.e. a genuine positive angle of attack.

    In ENU/FLU a nose-up pitch is a rotation of -theta about the body y-axis
    (which points LEFT), so the body-frame velocity has a NEGATIVE z component
    when the nose is up.
    """
    theta = math.radians(aoa_deg)
    R_body_to_world = Rotation.from_euler("y", -theta)
    v_body = R_body_to_world.inv().apply([speed, 0.0, 0.0])
    return FakeState(v_body, omega_flu)


def sideslip_state(speed, beta_deg):
    """Body state with a sideslip to the RIGHT of `beta_deg` (the standard
    positive sideslip): the velocity has a component along body -y in FLU."""
    b = math.radians(beta_deg)
    return FakeState([speed * math.cos(b), -speed * math.sin(b), 0.0])


# ---------------------------------------------------------------------------
FAILURES = []


def check(name, ok, detail=""):
    status = "PASS" if ok else "FAIL"
    print(f"  [{status}] {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(name)


def test_lift_curve_slope():
    """A wing lifts UP at positive angle of attack, and lifts more the more
    angle of attack it has.  (Pre-fix this was inverted: the model produced
    -24.5 N at +8 deg and +38.6 N at -8 deg.)"""
    print("\n1. lift-curve slope")
    aero = VTOLAerodynamics()
    fz = {}
    for a in (-8, -4, 0, 4, 8):
        f, _ = aero.update(level_flight_state(15.0, a))
        fz[a] = f[2]
        print(f"      AoA {a:+3d} deg -> F_z = {f[2]:+7.1f} N   "
              f"CL = {aero.last_state.coefficients['CL']:+5.2f}   "
              f"alpha_reported = {math.degrees(aero.last_state.alpha):+5.1f} deg")

    check("lift is UP at +6 deg AoA", fz[8] > 0, f"F_z={fz[8]:+.1f} N")
    check("lift is DOWN at -8 deg AoA", fz[-8] < 0, f"F_z={fz[-8]:+.1f} N")
    check("dF_z/dAoA > 0 (monotone lift curve)",
          fz[-8] < fz[-4] < fz[0] < fz[4] < fz[8])

    f, _ = aero.update(level_flight_state(15.0, 4.0))
    check("reported alpha matches the true AoA (standard sign)",
          abs(math.degrees(aero.last_state.alpha) - 4.0) < 0.05,
          f"reported {math.degrees(aero.last_state.alpha):+.2f} deg")


def test_static_pitch_stability():
    """Increasing angle of attack must produce a NOSE-DOWN moment.  In the FLU
    body frame +M_y IS nose-down, so M_y must increase with AoA."""
    print("\n2. static pitch stability")
    aero = VTOLAerodynamics()
    my = {}
    for a in (-6, 0, 6):
        _, m = aero.update(level_flight_state(15.0, a))
        my[a] = m[1]
        print(f"      AoA {a:+3d} deg -> M_y = {m[1]:+6.2f} N.m   (+M_y = nose-down in FLU)")
    check("restoring pitch moment (dM_nosedown/dAoA > 0)", my[-6] < my[0] < my[6])
    check("nose-down moment at +6 deg AoA", my[6] > 0, f"M_y={my[6]:+.2f}")


def test_pitch_damping():
    """A nose-down pitch RATE must produce a nose-up (opposing) moment.
    In FLU, +q is a nose-down rate, so M_y must go negative."""
    print("\n3. pitch damping")
    aero = VTOLAerodynamics()
    _, m_dn = aero.update(level_flight_state(15.0, 0.0, omega_flu=(0, math.radians(30), 0)))
    _, m_up = aero.update(level_flight_state(15.0, 0.0, omega_flu=(0, -math.radians(30), 0)))
    print(f"      q = +30 deg/s (nose-down rate) -> M_y = {m_dn[1]:+6.2f}")
    print(f"      q = -30 deg/s (nose-up rate)   -> M_y = {m_up[1]:+6.2f}")
    check("pitch rate is damped", m_dn[1] < 0 < m_up[1])


def test_pitch_rate_lift():
    """A nose-up pitch rate increases lift (CL_q > 0 in the standard sense)."""
    print("\n4. pitch-rate lift")
    aero = VTOLAerodynamics()
    f_up, _ = aero.update(level_flight_state(15.0, 0.0, omega_flu=(0, -math.radians(30), 0)))
    f_dn, _ = aero.update(level_flight_state(15.0, 0.0, omega_flu=(0, math.radians(30), 0)))
    print(f"      nose-up rate   -> F_z = {f_up[2]:+7.1f} N")
    print(f"      nose-down rate -> F_z = {f_dn[2]:+7.1f} N")
    check("nose-up rate makes more lift than nose-down rate", f_up[2] > f_dn[2])


def test_directional_stability():
    """Sideslip to the RIGHT must push the aircraft LEFT (side force) and yaw
    the nose RIGHT, into the relative wind (weathercock)."""
    print("\n5. side force + weathercock stability")
    aero = VTOLAerodynamics()
    f, m = aero.update(sideslip_state(15.0, 10.0))
    # FLU: +y is LEFT, +M_z is nose-LEFT.
    print(f"      beta = +10 deg (slip right) -> F_y = {f[1]:+6.1f} N (FLU, +y = left), "
          f"M_z = {m[2]:+6.2f} N.m (+ = nose-left)")
    check("side force opposes the sideslip (pushes left)", f[1] > 0)
    check("yaws the nose right, into the wind", m[2] < 0)


def test_dihedral_effect():
    """Sideslip to the RIGHT must roll the aircraft LEFT -- away from the
    sideslip.  A wing that rolls INTO its sideslip is spirally divergent.
    In FLU, +M_x rolls the RIGHT wing down."""
    print("\n6. dihedral effect")
    aero = VTOLAerodynamics()
    _, m_r = aero.update(sideslip_state(15.0, 10.0))
    _, m_l = aero.update(sideslip_state(15.0, -10.0))
    print(f"      beta = +10 deg (slip right) -> M_x = {m_r[0]:+6.2f} N.m (+ = right wing down)")
    print(f"      beta = -10 deg (slip left)  -> M_x = {m_l[0]:+6.2f} N.m")
    check("slip right rolls left (restoring dihedral)", m_r[0] < 0)
    check("dihedral effect is antisymmetric", m_l[0] > 0)


def test_roll_damping():
    """Roll rate must be opposed."""
    print("\n7. roll damping")
    aero = VTOLAerodynamics()
    _, m_p = aero.update(FakeState([15, 0, 0], [math.radians(60), 0, 0]))
    _, m_n = aero.update(FakeState([15, 0, 0], [-math.radians(60), 0, 0]))
    print(f"      p = +60 deg/s -> M_x = {m_p[0]:+6.2f}      p = -60 deg/s -> M_x = {m_n[0]:+6.2f}")
    check("roll rate is damped", m_p[0] < 0 < m_n[0])


def test_drag_opposes_motion():
    print("\n8. drag")
    aero = VTOLAerodynamics()
    for v in ([15, 0, 0], [0, 12, 0], [0, 0, -10], [-10, 0, 0]):
        f, _ = aero.update(FakeState(v))
        vv = np.array(v, dtype=float)
        along = float(np.dot(f, vv) / np.linalg.norm(vv))
        print(f"      v_body = {str(v):>14s} -> F = ({f[0]:+6.1f},{f[1]:+6.1f},{f[2]:+6.1f}) N, "
              f"component along velocity = {along:+6.1f} N")
        check(f"net force opposes motion for v_body={v}", along < 0, f"{along:+.1f} N")


def test_out_of_envelope_regimes():
    """A VTOL descends, slides sideways and backs up.  None of those may
    produce large forces in an absurd direction.  Pre-fix, backing up at
    10 m/s produced 15.4 N of DOWNFORCE (45% of weight) and a pure vertical
    descent produced a 10 N BACKWARD force."""
    print("\n9. non-fixed-wing regimes")
    aero = VTOLAerodynamics()
    weight = 3.45 * 9.81

    f, _ = aero.update(FakeState([-10.0, 0.0, 0.0]))       # backing up
    print(f"      backing up at 10 m/s      -> F = ({f[0]:+6.1f},{f[1]:+6.1f},{f[2]:+6.1f}) N")
    check("backing up: F_x is a forward-pointing drag", f[0] > 0)
    check("backing up: |F_z| < 10% of weight", abs(f[2]) < 0.10 * weight,
          f"{abs(f[2]):.1f} N vs {0.10 * weight:.1f} N")

    f, _ = aero.update(FakeState([0.0, 0.0, -10.0]))       # vertical descent
    print(f"      vertical descent 10 m/s   -> F = ({f[0]:+6.1f},{f[1]:+6.1f},{f[2]:+6.1f}) N")
    check("descent: force is upward (opposes the sink)", f[2] > 0)
    check("descent: |F_x| < 10% of weight", abs(f[0]) < 0.10 * weight,
          f"{abs(f[0]):.1f} N")

    f, _ = aero.update(FakeState([0.0, 8.0, 0.0]))         # pure sideways
    print(f"      sideways at 8 m/s         -> F = ({f[0]:+6.1f},{f[1]:+6.1f},{f[2]:+6.1f}) N")
    check("sideways: force opposes the slide", f[1] < 0)


def test_continuity():
    """The force must be continuous through the stall break -- an optimising
    controller will find and exploit any discontinuity.  A sharp stall is fine;
    a JUMP is not, so the bar is set relative to the linear-regime gradient."""
    print("\n10. continuity through stall")
    aero = VTOLAerodynamics()
    step = 0.5
    prev, worst, worst_at = None, 0.0, 0.0
    for a_deg in np.arange(-90.0, 90.01, step):
        f, _ = aero.update(level_flight_state(15.0, float(a_deg)))
        if prev is not None:
            d = float(np.linalg.norm(f - prev))
            if d > worst:
                worst, worst_at = d, float(a_deg)
        prev = f
    # gradient of the linear lift curve over the same alpha step, for scale
    f0, _ = aero.update(level_flight_state(15.0, 0.0))
    f1, _ = aero.update(level_flight_state(15.0, step))
    linear_step = float(np.linalg.norm(f1 - f0))
    print(f"      largest step {worst:.2f} N at AoA {worst_at:+.1f} deg; "
          f"linear-regime step {linear_step:.2f} N per {step} deg")
    check("no force jump exceeding 3x the linear-regime gradient",
          worst < 3.0 * linear_step, f"{worst / linear_step:.1f}x")


def test_stall_peaks_and_falls():
    print("\n11. stall behaviour")
    aero = VTOLAerodynamics()
    cls = []
    for a in range(0, 46, 3):
        aero.update(level_flight_state(15.0, float(a)))
        cls.append((a, aero.last_state.coefficients["CL"]))
    peak_a, peak_cl = max(cls, key=lambda t: t[1])
    print("      CL: " + "  ".join(f"{a}:{c:+.2f}" for a, c in cls))
    check("CL peaks near the stall angle", 8 <= peak_a <= 20, f"peak at {peak_a} deg")

    # Just past the break CL must actually DROP -- that is the stall.  Further
    # out the flat-plate branch legitimately rises again (a plate peaks near
    # 55 deg), so the property that matters in deep stall is not low lift but
    # a collapsed L/D: the wing stops being a wing and becomes a barn door.
    broke = min(c for a, c in cls if 15 <= a <= 24)
    check("CL breaks after the peak", broke < 0.6 * peak_cl,
          f"CL drops to {broke:.2f} from {peak_cl:.2f}")

    def ld(a):
        aero.update(level_flight_state(15.0, float(a)))
        c = aero.last_state.coefficients
        return c["CL"] / c["CD"]
    ld_cruise, ld_stalled = ld(6), ld(45)
    print(f"      L/D at 6 deg = {ld_cruise:5.1f}   at 45 deg = {ld_stalled:5.1f}")
    check("L/D collapses in deep stall", ld_stalled < 0.15 * ld_cruise,
          f"{ld_stalled:.2f} vs {ld_cruise:.1f}")


if __name__ == "__main__":
    print("VTOL aerodynamics -- physical-correctness regression tests")
    print("=" * 66)
    test_lift_curve_slope()
    test_static_pitch_stability()
    test_pitch_damping()
    test_pitch_rate_lift()
    test_directional_stability()
    test_dihedral_effect()
    test_roll_damping()
    test_drag_opposes_motion()
    test_out_of_envelope_regimes()
    test_continuity()
    test_stall_peaks_and_falls()
    print("\n" + "=" * 66)
    if FAILURES:
        print(f"{len(FAILURES)} CHECK(S) FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        sys.exit(1)
    print("all checks passed")
