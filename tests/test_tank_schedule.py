#!/usr/bin/env python3
"""The tank cascade's back-transition reference stays under the CL peak while the
wing is carrying the aircraft.

    ~/envs/isaac_env/bin/python3 tests/test_tank_schedule.py

WHY THIS IS PINNED (2026-09-13, gate + bag slosh_20260912_173754). The base
schedule adds a 15 deg nose-up flare from the first tick of the back-transition,
so at 12-14 m/s -- the tank aircraft's stall crossing, wing share 60-90% -- the
reference pitch was 14-18 deg on a wing whose CL peaks at 10. The outer solver
carries that lift curve, and past the peak its pitch-vs-lift gradient flips:
the solution alternated between following the pitch reference and holding
altitude, a +-7 deg ring at ~1.2 Hz that zero latency does not cure. Under
0.1 deg of attitude noise the ring ran to AoA 20+ on 2 of 16 seeds, the solve
failed, the fallback held the elevator hard nose-up and the aircraft departed
-- the flown crash. Releasing the flare only once the rotors carry the load
(W_FLARE, on the aft-tilt gate) took the ring out entirely: pitch error rms
8.2 -> 2.5 deg, solve failures 3 -> 0, stop time unchanged (the flare was not
braking there anyway), and the noise sweeps went 14/16 -> 16/16 at every
stress level tried. This file fails if the release ever creeps back toward
the wing-borne end.

The numbers here come from the schedule itself, not from a solver.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir              # noqa: E402

sys.path.insert(0, controller_dir())
from px4_vtol_direct_actuator import nmpc_cascade_node as base           # noqa: E402
from px4_vtol_direct_actuator import nmpc_cascade_tank_node as tank      # noqa: E402
from px4_vtol_direct_actuator import nmpc6_cruise_node as n6             # noqa: E402

N, DT, PERIOD = 14, 0.1, 0.05
VC, TTR, Z = 16.0, 20.0, 30.0
FLARE, AFT = math.radians(15.0), math.radians(20.0)
CL_PEAK_DEG = 10.0


def refs_at(v_cmd, decel=True, th_prev=None, b_prev=None, fn=None):
    fn = fn or tank.build_refs_tank
    sched = n6.Schedule(V_cruise=VC, tilt_cap_deg=90.0, alpha_max_deg=8.0)
    out, b, th = fn(sched, N, DT, PERIOD, v_cmd, VC, TTR, Z, decel=decel,
                    decel_rate=1.5, flare=FLARE, beta_aft=AFT, b_slew=60.0,
                    b_prev=b_prev, th_slew=8.0, th_prev=th_prev)
    return out


def test_forward_is_the_base_schedule():
    for v in (0.0, 4.0, 9.0, 14.0, 16.0):
        a = refs_at(v, decel=False)
        b = refs_at(v, decel=False, fn=base.build_refs)
        for ra, rb in zip(a, b):
            for k in ra:
                assert abs(ra[k] - rb[k]) < 1e-12, (v, k, ra[k], rb[k])


def test_flare_held_off_while_wing_borne():
    """No stage of the horizon commands more than aoa_cap while the wing carries
    more than W_FLARE of the weight -- with NO rate limiter helping (th_prev=None),
    so this is the schedule's own ceiling, not the slew."""
    assert tank.AOA_CAP_DEG <= CL_PEAK_DEG, "the cap sits at or under the CL peak"
    assert tank.W_FLARE <= tank.W_AFT + 1e-9, "the flare releases no earlier than the aft tilt"
    v_release = base.V_STALL * math.sqrt(tank.W_FLARE)
    for v_cmd in (16.0, 15.5, 15.0, 14.0, 13.0, 12.5):
        for r in refs_at(v_cmd):
            if r["vs"] > v_release:
                assert math.degrees(r["theta"]) <= tank.AOA_CAP_DEG + 1e-9, (v_cmd, r["vs"], math.degrees(r["theta"]))
                assert abs(r["beta"]) < 1e-9, "rotors upright while wing-borne"


def test_flare_and_aft_tilt_fully_released_at_hover():
    r = refs_at(0.5)[0]
    # rotor-borne: the whole flare is available (V/Vc fades it, so at V~0 the
    # schedule's own theta+flare*frac term is what binds, and it is below the cap)
    wing = (r["vs"] / base.V_STALL) ** 2
    th_max = math.radians(tank.AOA_CAP_DEG) + FLARE * min(1.0, (tank.W_FLARE - wing) / tank.W_FLARE)
    assert th_max >= math.radians(tank.AOA_CAP_DEG + 14.9)
    r8 = refs_at(8.0)[0]
    assert math.degrees(r8["theta"]) > tank.AOA_CAP_DEG, "below the gate the flare is released"


def test_w_flare_one_recovers_the_old_release():
    old = tank.W_FLARE
    try:
        tank.W_FLARE = 1.0
        r = refs_at(14.0)[0]
        wing = (14.0 / base.V_STALL) ** 2
        expect = min(math.radians(8.0) + FLARE * 14.0 / VC,
                     math.radians(tank.AOA_CAP_DEG) + (1.0 - wing) * FLARE)
        assert abs(r["theta"] - expect) < 1e-6
    finally:
        tank.W_FLARE = old


def test_reference_is_still_a_trim():
    """The re-solved (Tf, Tr) balance the commanded (beta, theta) at every stage."""
    for v_cmd in (15.0, 12.0, 8.0, 3.0):
        for r in refs_at(v_cmd):
            V = max(r["vs"], 1.0)
            Tf, Tr = base.so3.trim_split(r["beta"], r["theta"], V, r["de"],
                                         base.so3.M * base.so3.G, 0.0)
            assert abs(float(Tf) - r["Tf"]) < 1e-6 and abs(float(Tr) - r["Tr"]) < 1e-6, (v_cmd, r)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    print(f"tank schedule: cap {tank.AOA_CAP_DEG:.0f} deg, flare release below wing share "
          f"{tank.W_FLARE:.2f} (V < {base.V_STALL * math.sqrt(tank.W_FLARE):.1f} m/s), "
          f"aft tilt below {tank.W_AFT:.2f}")
    sys.exit(1 if fails else 0)
