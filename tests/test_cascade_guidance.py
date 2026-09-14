#!/usr/bin/env python3
"""The cascade's guidance layer: legs with fly-by turns, the orbit, return-to-home.

    ~/envs/isaac_env/bin/python3 tests/test_cascade_guidance.py

`nmpc_cascade_node.Guidance` is shared by the node and the offline gate and driven by one
text protocol ("wp X Y ...", "loiter X Y R [dir]", "rth", "hold"). What is pinned here is the
geometry the 2026-09-13 gate runs established: a leg ends at the fly-by point
R_turn*tan(dpsi/2) + 1.5 V; the last leg hands over to an orbit at the tangent-join distance
R_turn + sqrt(R^2 - R_turn^2) + 1.5 V; the orbit's field turns the commanded direction and
carries the coordinated bank as a feed-forward (a P bank law otherwise holds an 18 deg bank
only with 15 deg of standing heading error -- the orbit sat 8 m outside its circle);
return-to-home brakes at V^2/(2 a) + margin from home.

2026-09-13 evening: the orbit also carries its own turn rate (so the bank law's rate damping stops
opposing the turn: it was taking 8.9 deg of bank back off the feed-forward and the field held
3.4 m of radius error to pay for it) and an INTEGRAL of the radius error, clamped and banded
(gate: rho 103.3 -> 100.0 at R 100); and the rth refuses to arm the back-transition in a bank
over roll_arm_max, down to the no-margin braking distance, where it arms regardless.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir              # noqa: E402

sys.path.insert(0, controller_dir())
from px4_vtol_direct_actuator import nmpc_cascade_node as base   # noqa: E402

G = 9.81
V = 16.0


def guidance(**kw):
    kw.setdefault("brake_margin", 15.0)
    g = base.Guidance(k_ct=0.03, ct_max=math.radians(35), bank_max=math.radians(25),
                      loiter_radius=100.0, decel_rate=1.5, **kw)
    g.home = np.array([0.0, 0.0])
    return g


def test_protocol():
    g = guidance()
    assert g.command("wp 500 0 500 400", (0.0, 0.0)).startswith("legs: 2")
    assert g.mode == "leg" and len(g.wps) == 2
    assert g.command("loiter 100 400 80 -1", (0.0, 0.0)).startswith("loiter")
    assert g.mode == "loiter" and g.dir == -1 and g.radius == 80.0
    assert g.command("rth", (300.0, 300.0)).startswith("rth")
    assert g.mode == "rth" and np.allclose(g.wps[0], [0.0, 0.0])
    assert "needs" in g.command("wp 1", (0.0, 0.0))
    assert "unknown" in g.command("dance", (0.0, 0.0))
    g.home = None
    assert "no home" in g.command("rth", (0.0, 0.0))


def test_turn_radius_is_the_bank_limit():
    g = guidance()
    assert abs(g.turn_radius(V) - V * V / (G * math.tan(math.radians(25)))) < 1e-9   # 56 m


def test_leg_switches_at_the_fly_by_point():
    g = guidance(); g.command("wp 500 0 500 400", (0.0, 0.0))
    R_t = g.turn_radius(V)
    antic = R_t * math.tan(math.radians(90) / 2.0) + 1.5 * V      # 56 + 24 = 80 m
    # well before the corner: still leg 1, heading east
    hd = g.update((300.0, 0.0), (V, 0.0), 0.0, 0.0, V)
    assert len(g.wps) == 2 and abs(hd) < 1e-9
    # just outside the fly-by distance: still leg 1
    g.update((500.0 - antic - 1.0, 0.0), (V, 0.0), 0.0, 0.0, V)
    assert len(g.wps) == 2
    # inside it: leg 2, and the command swings toward north
    hd = g.update((500.0 - antic + 1.0, 0.0), (V, 0.0), 0.0, 0.0, V)
    assert len(g.wps) == 1 and hd > math.radians(30)


def test_last_leg_hands_over_to_an_orbit_at_the_join_distance():
    g = guidance(); g.command("wp 100 400", (500.0, 400.0))
    R_t = g.turn_radius(V); R = 100.0
    d_join = R_t + math.sqrt(R * R - R_t * R_t) + 1.5 * V
    g.update((100.0 + d_join + 2.0, 400.0), (-V, 0.0), math.pi, math.pi, V)
    assert g.mode == "leg"
    g.update((100.0 + d_join - 2.0, 400.0), (-V, 0.0), math.pi, math.pi, V)
    assert g.mode == "loiter" and np.allclose(g.center, [100.0, 400.0]) and g.radius == 100.0


def test_orbit_field_and_feed_forward():
    g = guidance(); g.command("loiter 0 0 100 1", (200.0, 0.0))
    # on the circle, east point, counter-clockwise: the tangent is north, bank is a LEFT roll
    hd = g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V)
    assert abs(base._wrap(hd - math.pi / 2)) < math.radians(1)
    assert g.bank_ff < 0 and abs(abs(g.bank_ff) - math.atan(V * V / (G * 100.0))) < 1e-9
    # outside the circle the field points inward of the tangent; inside, outward
    hd_out = g.update((150.0, 0.0), (0.0, V), math.pi / 2, 0.0, V)
    hd_in = g.update((60.0, 0.0), (0.0, V), math.pi / 2, 0.0, V)
    assert hd_out > math.pi / 2 + math.radians(5) and hd_in < math.pi / 2 - math.radians(5)
    # clockwise flips the tangent and the bank
    g.command("loiter 0 0 100 -1", (200.0, 0.0))
    hd = g.update((100.0, 0.0), (0.0, -V), -math.pi / 2, 0.0, V)
    assert abs(base._wrap(hd + math.pi / 2)) < math.radians(1) and g.bank_ff > 0
    # a radius under the bank limit's is raised to it
    g.command("loiter 0 0 20 1", (200.0, 0.0))
    g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V)
    assert abs(abs(g.bank_ff) - math.atan(V * V / (G * g.turn_radius(V) * 1.15))) < 1e-9


def test_orbit_turn_rate_feed_forward_and_radius_integral():
    g = guidance(k_i_orbit=0.002, i_orbit_max=math.radians(6.0), i_orbit_band=10.0)
    g.command("loiter 0 0 100 1", (200.0, 0.0))
    # the orbit's own heading rate, CCW positive like r = w[2]; clockwise flips it
    g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V)
    assert abs(g.turn_rate_ff - V / 100.0) < 1e-9
    # no dt: nothing integrates (the tests above rely on this)
    assert g.i_orbit == 0.0
    ff0 = abs(g.bank_ff)
    # 3.4 m OUTSIDE for 10 s asks for MORE bank in the turn's direction (CCW: more negative)
    for _ in range(100):
        g.update((103.4, 0.0), (0.0, V), math.pi / 2, 0.0, V, dt=0.1)
    assert abs(g.i_orbit - 34.0) < 1e-6
    assert g.bank_ff < 0 and abs(g.bank_ff) - ff0 > math.radians(3.5)
    # clamped at i_orbit_max of bank
    for _ in range(2000):
        g.update((103.4, 0.0), (0.0, V), math.pi / 2, 0.0, V, dt=0.1)
    assert abs(abs(g.bank_ff) - ff0 - math.radians(6.0)) < 1e-9
    # outside the band (the entry cuts 14 m in) it does not integrate
    i_before = g.i_orbit
    g.update((85.0, 0.0), (0.0, V), math.pi / 2, 0.0, V, dt=0.1)
    assert g.i_orbit == i_before
    # a new loiter command resets it; clockwise carries the opposite sign
    g.command("loiter 0 0 100 -1", (200.0, 0.0))
    assert g.i_orbit == 0.0
    g.update((100.0, 0.0), (0.0, -V), -math.pi / 2, 0.0, V, dt=0.1)
    assert abs(g.turn_rate_ff + V / 100.0) < 1e-9 and g.bank_ff > 0
    # off the orbit nothing is fed forward
    g.command("hold", (0.0, 0.0)); g.update((0.0, 0.0), (V, 0.0), 0.0, 0.0, V, dt=0.1)
    assert g.bank_ff == 0.0 and g.turn_rate_ff == 0.0


def test_adapted_bank_feed_forward_learns_kappa_in_a_steady_bank():
    G_ = base.so3.G
    g = guidance(k_i_orbit=0.0, bank_adapt=True, bank_adapt_tau=2.0)
    g.command("loiter 0 0 100 1", (200.0, 0.0))
    phi = math.radians(-18.0)                                   # a left bank (CCW orbit)
    psi_c = -G_ * math.tan(phi) / V                             # coordinated: > 0 (CCW)
    assert psi_c > 0
    ff0 = abs(g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V) and g.bank_ff)
    # the plant turns at 0.8 x the coordinated rate: kappa -> 0.8, and the orbit asks MORE bank
    for _ in range(200):
        g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V, roll=phi, dt=0.05,
                 psi_dot=0.8 * psi_c, roll_rate=0.0)
    assert abs(g.kappa - 0.8) < 0.01, g.kappa
    assert abs(abs(g.bank_ff) - math.atan(V * V / (G_ * 100.0 * g.kappa))) < 1e-9
    assert abs(g.bank_ff) > ff0 + math.radians(3.0)
    # the sign convention is symmetric: a right bank turning right at the same ratio
    g2 = guidance(k_i_orbit=0.0, bank_adapt=True, bank_adapt_tau=2.0)
    g2.command("loiter 0 0 100 -1", (200.0, 0.0))
    for _ in range(200):
        g2.update((100.0, 0.0), (0.0, -V), -math.pi / 2, 0.0, V, roll=-phi, dt=0.05,
                  psi_dot=-0.8 * psi_c, roll_rate=0.0)
    assert abs(g2.kappa - 0.8) < 0.01 and g2.bank_ff > 0
    # gates: no learning while rolling in, at a shallow bank, or below wing-borne speed
    for kw in (dict(roll=phi, roll_rate=math.radians(20.0), psi_dot=0.5 * psi_c),
               dict(roll=math.radians(-3.0), roll_rate=0.0, psi_dot=0.5 * psi_c)):
        k0 = g.kappa; g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V, dt=0.05, **kw)
        assert g.kappa == k0, kw
    k0 = g.kappa; g.update((100.0, 0.0), (0.0, 10.0), math.pi / 2, 0.0, 10.0, roll=phi, dt=0.05, psi_dot=0.5 * psi_c)
    assert g.kappa == k0
    # clamped: a wild sample cannot take kappa outside [0.6, 1.3]
    for _ in range(400):
        g.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V, roll=phi, dt=0.05, psi_dot=0.1 * psi_c)
    assert abs(g.kappa - 0.6) < 1e-3
    # it learns in ANY mode (the fly-by turns teach it before the orbit), and off means 1.0
    g3 = guidance(bank_adapt=True); g3.command("wp 500 0 500 400", (0.0, 0.0))
    for _ in range(100):
        g3.update((300.0, 0.0), (V, 0.0), 0.0, 0.0, V, roll=math.radians(25.0), dt=0.05,
                  psi_dot=-0.8 * G_ * math.tan(math.radians(25.0)) / V)
    assert g3.kappa < 0.9 and g3.kappa_n == 100
    g4 = guidance(bank_adapt=False)
    g4.update((100.0, 0.0), (0.0, V), math.pi / 2, 0.0, V, roll=phi, dt=0.05, psi_dot=0.8 * psi_c)
    assert g4.kappa == 1.0 and g4.kappa_n == 0


def test_rth_does_not_arm_in_a_bank_but_never_past_the_no_margin_distance():
    g = guidance(brake_margin=5.0, roll_arm_max=math.radians(3.0)); g.command("rth", (400.0, 0.0))
    d_stop = V * V / (2 * 1.5)                                  # 85.3 m
    # inside the margin, 6.6 deg banked (the 0.7 kg flight): held off
    g.update((d_stop + 4.0, 0.0), (-V, 0.0), math.pi, 0.0, V, roll=math.radians(-6.6), dt=0.5)
    assert not g.want_decel and g.arm_held == 0.5
    # wings level: arms, and says how long it waited
    g.update((d_stop + 3.0, 0.0), (-V, 0.0), math.pi, 0.0, V, roll=math.radians(2.0), dt=0.5)
    assert g.want_decel and abs(g.arm_roll - math.radians(2.0)) < 1e-9 and "held 0.5 s" in g.status
    # a roll that never settles: arms roll_wait_m past the no-margin distance regardless
    g = guidance(brake_margin=5.0, roll_wait_m=10.0); g.command("rth", (400.0, 0.0))
    g.update((d_stop - 9.0, 0.0), (-V, 0.0), math.pi, 0.0, V, roll=math.radians(8.0), dt=0.5)
    assert not g.want_decel
    g.update((d_stop - 11.0, 0.0), (-V, 0.0), math.pi, 0.0, V, roll=math.radians(8.0), dt=0.5)
    assert g.want_decel and abs(g.arm_roll - math.radians(8.0)) < 1e-9
    # ...and if that overruns home, the nose stays on the latched bearing: no 180 deg swing
    hd = g.update((20.0, 0.0), (-V, 0.0), math.pi, 0.0, 6.0, roll=0.0, dt=0.5)
    assert abs(base._wrap(hd - math.pi)) < 1e-9
    hd = g.update((-8.0, 0.5), (-6.0, 0.0), math.pi, 0.0, 6.0, roll=0.0, dt=0.5)
    assert abs(base._wrap(hd - math.pi)) < 1e-9
    # roll_arm_max 0 = the guard off: arms banked, at the margin (the pre-2026-09-13 evening behaviour)
    g = guidance(brake_margin=5.0, roll_arm_max=0.0); g.command("rth", (400.0, 0.0))
    g.update((d_stop + 4.0, 0.0), (-V, 0.0), math.pi, 0.0, V, roll=math.radians(8.0), dt=0.5)
    assert g.want_decel and g.arm_held == 0.0


def test_rth_brakes_at_the_braking_distance_and_keeps_the_nose_on_home():
    g = guidance(); g.command("rth", (400.0, 0.0))
    d_brake = V * V / (2 * 1.5) + 15.0                          # 100 m
    g.update((d_brake + 5.0, 0.0), (-V, 0.0), math.pi, 0.0, V)
    assert not g.want_decel
    hd = g.update((d_brake - 5.0, 0.0), (-V, 0.0), math.pi, 0.0, V)
    assert g.want_decel and abs(base._wrap(hd - math.pi)) < 1e-9
    # past home while stopping (V small): the nose stays where it is, no 180 deg swing
    hd = g.update((-2.0, 0.5), (-1.0, 0.0), math.pi, 0.0, 1.0)
    assert abs(base._wrap(hd - math.pi)) < 1e-9


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"PASS {name}")
            except AssertionError as e:
                fails += 1; print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
