#!/usr/bin/env python3
"""Is the aerodynamic coefficient set consistent with the airframe it describes?

The derivatives in `VTOLAerodynamicsConfig` were imported wholesale from the AVTOL design
(a 1325 kg aircraft) and re-scaled by geometry. Some of them did not survive the transfer:
`cl_aileron` was 6x below what the modelled aileron panel can produce, and `cn_beta` sat
above the fin-alone value where a fuselage can only push it down. Neither error is visible
in a flight test until a controller runs out of authority on one axis (2026-09-06 audit).

Two kinds of check here:

  * GEOMETRY  -- each control derivative against a first-principles estimate from the
                 surface that quad_tiltrotor_config actually builds. These carry real
                 uncertainty, so the bar is a factor of two.
  * IDENTITY  -- relations that hold by definition, with no estimate involved. The clearest:
                 a fin's yaw moment IS its side force times its arm, so Cn/CY = l_v/b for
                 both the sideslip pair and the rudder pair, and the two pairs must agree.

    ~/envs/isaac_env/bin/python3 tests/test_aero_consistency.py
"""
import importlib.util
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "vtol_aerodynamics",
    os.path.join(_HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                 "logic", "dynamics", "vtol_aerodynamics.py"))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
CFG = _mod.VTOLAerodynamicsConfig()

# ---------------------------------------------------------------------------
# Surface geometry, mirroring quad_tiltrotor_config() in vtol.py. If an airframe
# changes there, change it here -- that is the point of the test.
# ---------------------------------------------------------------------------
CG_X = 0.176                                   # assembled vehicle CG (tests/plant_double.py)
ELEVATOR = dict(span=0.45, chord=0.14, x=-0.45)
RUDDER = dict(span=0.22, chord=0.14, x=-0.45)
AILERON = dict(span=0.35, chord=0.12, y_centre=0.60)
FLAP_EFFECTIVENESS = 0.65                      # tau at cf/c ~ 0.5

FAILURES = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(name)


def lift_slope(aspect_ratio):
    return 2.0 * math.pi * aspect_ratio / (aspect_ratio + 2.0)


def within(have, estimate, factor=2.0):
    if abs(estimate) < 1e-9:
        return False
    r = abs(have / estimate)
    return (1.0 / factor) <= r <= factor


def geometry():
    S, b, c = CFG.wing_area, CFG.wing_span, CFG.mean_chord
    lt = abs(ELEVATOR["x"] - CG_X)
    St = ELEVATOR["span"] * ELEVATOR["chord"]
    at = lift_slope(ELEVATOR["span"] / ELEVATOR["chord"])
    lv = abs(RUDDER["x"] - CG_X)
    Sv = RUDDER["span"] * RUDDER["chord"]
    av = lift_slope(RUDDER["span"] / RUDDER["chord"])
    y2 = min(b / 2.0, AILERON["y_centre"] + AILERON["span"] / 2.0)
    y1 = max(0.0, AILERON["y_centre"] - AILERON["span"] / 2.0)
    cl_da = (2.0 * CFG.cl_alpha * FLAP_EFFECTIVENESS / (S * b)) * c * (y2 ** 2 - y1 ** 2) / 2.0
    return dict(S=S, b=b, c=c, lt=lt, St=St, at=at, lv=lv, Sv=Sv, av=av,
                VH=St * lt / (S * c), VV=Sv * lv / (S * b), cl_da=cl_da, y1=y1, y2=y2)


def test_geometry_estimates():
    print("\n1. control derivatives vs the surfaces the airframe actually has")
    g = geometry()
    print(f"      tail volume V_H = {g['VH']:.3f} (a_t={g['at']:.2f});  "
          f"fin volume V_V = {g['VV']:.4f} (a_v={g['av']:.2f})")
    print(f"      aileron spans y {g['y1']:.3f}..{g['y2']:.3f} m of a {g['b']/2:.2f} m semi-span, "
          f"cf/c = {AILERON['chord']/g['c']:.2f}")
    rows = [
        ("Cm_de  elevator power", CFG.cm_elevator, -g["VH"] * g["at"]),
        ("Cn_dr  rudder power", CFG.cn_rudder, -g["VV"] * g["av"]),
        ("Cl_da  aileron power", CFG.cl_aileron, g["cl_da"]),
        ("Cl_p   roll damping", CFG.cl_p, -CFG.cl_alpha / 6.0),
    ]
    print(f"      {'derivative':24s} {'in file':>9s} {'geometry':>10s} {'ratio':>7s}")
    for name, have, est in rows:
        r = have / est
        print(f"      {name:24s} {have:9.3f} {est:10.3f} {r:7.2f}")
        check(f"{name.split()[0]} is within a factor of 2 of its geometry", within(have, est),
              f"{have:.3f} vs {est:.3f}")


def test_fin_identity():
    """A fin's yaw moment is its side force times its arm. Nothing to estimate."""
    print("\n2. fin identity: Cn/CY must equal l_v/b for the SAME surface")
    g = geometry()
    target = g["lv"] / g["b"]
    r_beta = abs(CFG.cn_beta / CFG.cy_beta)
    r_rud = abs(CFG.cn_rudder / CFG.cy_rudder)
    print(f"      l_v / b                = {target:5.2f}")
    print(f"      |Cn_beta| / |CY_beta|  = {r_beta:5.2f}   "
          f"(a fuselage lowers this: it adds side force with little yaw moment)")
    print(f"      |Cn_dr|   / |CY_dr|    = {r_rud:5.2f}")
    # sideslip may sit BELOW l_v/b (fuselage side area), but never above it
    check("sideslip pair does not claim more yaw moment than the fin arm allows",
          r_beta <= target * 1.15, f"{r_beta:.2f} vs {target:.2f}")
    check("rudder pair is within a factor of 2 of the fin arm",
          0.5 * target <= r_rud <= 2.0 * target, f"{r_rud:.2f} vs {target:.2f}")


def test_weathercock_below_fin_alone():
    print("\n3. weathercock stability cannot exceed the fin that produces it")
    g = geometry()
    fin_alone = g["VV"] * g["av"]
    print(f"      Cn_beta in file = {CFG.cn_beta:.3f};  fin alone = {fin_alone:.3f}  "
          f"(a fuselage only reduces it)")
    check("Cn_beta <= the fin-alone value", CFG.cn_beta <= fin_alone * 1.15,
          f"{CFG.cn_beta:.3f} vs {fin_alone:.3f}")


def test_authority_handover():
    """Every axis must have a speed where the aero surface can take over from the rotors,
    below the speed at which the rotors are asked to tilt away and stop providing it."""
    print("\n4. does each axis have a hand-over from rotors to surfaces?")
    S, b, c = CFG.wing_area, CFG.wing_span, CFG.mean_chord
    dmax = CFG.max_surface_deflection_rad
    # rotor authority at tilt 0, +-9 N differential on the CAD arms (plant_double geometry)
    rot = dict(roll=2 * 9.0 * 0.284, pitch=2 * 9.0 * 0.2855, yaw=0.12 * 4 * 9.0 / 2)
    surf = dict(roll=(b, abs(CFG.cl_aileron)), pitch=(c, abs(CFG.cm_elevator)),
                yaw=(b, abs(CFG.cn_rudder)))
    stall_v = math.sqrt(2 * 3.45 * 9.81 / (CFG.rho * S * 1.2))
    print(f"      stall ~ {stall_v:.1f} m/s; wing-borne cruise 14-18 m/s")
    for axis in ("roll", "pitch", "yaw"):
        arm, coeff = surf[axis]
        k = 0.5 * CFG.rho * S * arm * coeff * dmax
        v_cross = math.sqrt(rot[axis] / k) if k > 1e-9 else float("inf")
        print(f"      {axis:5s}: surfaces match the rotors at {v_cross:5.1f} m/s "
              f"(rotor {rot[axis]:.2f} N.m)")
        check(f"{axis} hands over below wing-borne speed", v_cross < 14.0,
              f"cross-over {v_cross:.1f} m/s")


if __name__ == "__main__":
    print("aerodynamic coefficient set vs the airframe geometry")
    print("=" * 70)
    test_geometry_estimates()
    test_fin_identity()
    test_weathercock_below_fin_alone()
    test_authority_handover()
    print("\n" + "=" * 70)
    if FAILURES:
        print(f"{len(FAILURES)} CHECK(S) FAILED:")
        for f in FAILURES:
            print("  -", f)
        sys.exit(1)
    print("all checks passed")
