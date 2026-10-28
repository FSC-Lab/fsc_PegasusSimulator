"""Rotor angle from the bag npz, and local-polynomial rate/acceleration."""
import numpy as np

def load(path, topic="vrpn"):
    d = np.load(path)
    t = d[topic + "__header.stamp.sec"] + 1e-9 * d[topic + "__header.stamp.nanosec"]
    qx, qy, qz, qw = [d[topic + "__pose.orientation." + c] for c in "xyzw"]
    p = np.c_[d[topic + "__pose.position.x"], d[topic + "__pose.position.y"], d[topic + "__pose.position.z"]]
    th = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))
    return t - t[0], th, p

def local_poly(t, y, half_window_s, deg=2):
    """Value, 1st, 2nd derivative of a local polynomial fit centred on each sample."""
    n = len(t); v = np.full(n, np.nan); d1 = np.full(n, np.nan); d2 = np.full(n, np.nan)
    lo = np.searchsorted(t, t - half_window_s); hi = np.searchsorted(t, t + half_window_s, side="right")
    for i in range(n):
        s = slice(lo[i], hi[i])
        if hi[i] - lo[i] < deg + 3:
            continue
        x = t[s] - t[i]
        c = np.polyfit(x, y[s], deg)
        v[i] = c[-1]; d1[i] = c[-2]; d2[i] = 2 * c[-3] if deg >= 2 else 0.0
    return v, d1, d2
