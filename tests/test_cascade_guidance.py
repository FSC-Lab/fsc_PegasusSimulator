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


def guidance():
    g = base.Guidance(k_ct=0.03, ct_max=math.radians(35), bank_max=math.radians(25),
                      loiter_radius=100.0, decel_rate=1.5, brake_margin=15.0)
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
