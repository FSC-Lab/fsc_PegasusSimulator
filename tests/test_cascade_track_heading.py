#!/usr/bin/env python3
"""The cascade's cross-track law must not steer toward a hover offset.

    ~/envs/isaac_env/bin/python3 tests/test_cascade_track_heading.py

Bag slosh_20260913_102448 (2026-09-13): the flight driver engages the cascade
and publishes the far target ~3 s later. In between, the reference is still
the climb's hover point, 4 mm from where the cascade latched its start point,
and `track_heading` computed a track BEARING from that 4 mm vector -- a random
direction, ~160 deg that day. The yaw loop drove the aircraft round at full
authority while the settle gate had already begun the acceleration; the target
arrived with the aircraft flying backwards at 3 m/s and 32 deg of tilt, and it
departed in roll. The guard was 1 mm; the hover holds to ~6 mm. Every earlier
flight had passed the same race by luck. A track shorter than `min_track` is
a hover point: the latched heading stands.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir              # noqa: E402

sys.path.insert(0, controller_dir())
from px4_vtol_direct_actuator import nmpc_cascade_node as base   # noqa: E402

K_CT, CT_MAX = 0.05, math.radians(35.0)


def test_millimetre_baseline_keeps_the_latched_heading():
    # the flown case: hover reference (0.02, 0.01), engage point (0.024, 0.0084)
    for latched in (0.0, 0.3, -2.0):
        h, e = base.track_heading((0.024, 0.0084), (0.02, 0.01), (0.024, 0.0084), latched, K_CT, CT_MAX)
        assert h == latched and e == 0.0, (latched, h, e)
    # anything under min_track, anywhere on the compass
    for ang in range(0, 360, 45):
        tgt = (3.0 * math.cos(math.radians(ang)), 3.0 * math.sin(math.radians(ang)))
        h, e = base.track_heading((0.0, 0.0), tgt, (0.0, 0.0), 0.7, K_CT, CT_MAX)
        assert h == 0.7 and e == 0.0, (ang, h)


def test_real_track_gives_its_bearing_and_cross_track():
    h, e = base.track_heading((0.024, 0.0084), (800.02, 0.01), (0.024, 0.0084), 0.0, K_CT, CT_MAX)
    assert abs(h) < 1e-4 and abs(e) < 1e-3
    # 10 m LEFT of a track along +x -> steer right (heading decreases), by k_ct * e_ct
    h, e = base.track_heading((100.0, 10.0), (800.0, 0.0), (0.0, 0.0), 0.0, K_CT, CT_MAX)
    assert abs(e - 10.0) < 1e-6 and abs(h - (-K_CT * 10.0)) < 1e-6
    # the correction is capped at ct_max
    h, e = base.track_heading((100.0, 200.0), (800.0, 0.0), (0.0, 0.0), 0.0, K_CT, CT_MAX)
    assert abs(h + CT_MAX) < 1e-6
    # a target BEHIND is a real track: bearing pi (the mission is a straight line from p0)
    h, e = base.track_heading((0.0, 0.0), (-800.0, 0.0), (0.0, 0.0), 0.0, K_CT, CT_MAX)
    assert abs(abs(h) - math.pi) < 1e-6


def test_min_track_is_a_real_distance():
    import inspect
    sig = inspect.signature(base.track_heading)
    assert sig.parameters["min_track"].default >= 2.0, "a hover offset is centimetres; the guard must be metres"


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"PASS {name}")
            except AssertionError as e:
                fails += 1; print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
