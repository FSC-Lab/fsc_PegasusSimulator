"""Shared helpers for the 2026-10-21 box-pushing flight analysis."""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.path.join(HERE, "..", "npz")
BAGS = ["161745", "162604", "164338", "165311", "170123"]
NAV = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER", 5: "RTL", 10: "ACRO",
       12: "DESCEND", 13: "TERM", 14: "OFFBOARD", 15: "STAB", 17: "TAKEOFF", 18: "LAND"}
STEPS = ["go_to_start", "ready", "push", "exit", "go_to_land_start", "land"]
GUARD = {0: "MOCAP", 1: "BRIDGING", 2: "CONFIRMED", 3: "LOST"}


class Bag:
    def __init__(self, tag):
        self.tag = tag
        self.z = np.load(os.path.join(NPZ, f"b{tag}.npz"), allow_pickle=True)
        # time base: the first receive time of any topic in the bag
        self.t0 = min(self.z[k][0] for k in self.z.files if k.endswith("__recv") and len(self.z[k]))

    def has(self, key):
        return f"{key}__recv" in self.z.files

    def t(self, key):
        return self.z[f"{key}__recv"] - self.t0

    def f(self, key, field):
        return self.z[f"{key}__{field}"]

    def series(self, key, *fields):
        return self.t(key), [self.f(key, f) for f in fields]

    def edges(self, key, field="data"):
        """(t, value) at every change of a string/number stream."""
        if not self.has(key):
            return []
        t, v = self.t(key), self.f(key, field)
        out, last = [], None
        for ti, vi in zip(t, v):
            s = str(vi)
            if s != last:
                out.append((ti, s))
                last = s
        return out


def tilt_deg(qw, qx, qy, qz):
    r22 = 1.0 - 2.0 * (qx * qx + qy * qy)
    return np.degrees(np.arccos(np.clip(r22, -1, 1)))


def yaw_deg(qw, qx, qy, qz):
    return np.degrees(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


def window(t, t0, t1):
    return (t >= t0) & (t <= t1)
