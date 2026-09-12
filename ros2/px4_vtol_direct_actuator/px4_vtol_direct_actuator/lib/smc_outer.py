"""Outer-loop sliding-mode position controller (shared by the direct_actuator + attitude
controllers). It augments the VELOCITY reference to control POSITION robustly:

    e   = sp - target                      (set-point error to the target)
    v   = -lambda*e - K*sat(e/phi)         (reaching term + boundary-layer sliding term)
    sp += v*dt   (advanced toward the target, clamped so it never overshoots)

`sp` is a smooth, CONVERGING, stationary reference (feeding a moving lead-point that tracks
the vehicle wrecks an NMPC warm-start); `v` is the robust velocity feed-forward. Two regimes,
selected by the TARGET altitude:
  * LOOSE  (cruise / transition)  -> big v_max, wide boundary layer -> position may change fast
  * STRICT (landing)              -> small v_max, tight boundary layer -> precise, slow touchdown
"""
import math
import numpy as np

_KEYS = ("land_z", "lam", "K", "phi", "vmax", "vzmax",
         "lam_land", "K_land", "phi_land", "vmax_land", "vzmax_land", "reset_dist", "amax")


def default_params():
    return dict(
        land_z=1.2, reset_dist=8.0, amax=100.0,   # amax = accel limit on the velocity cmd [m/s^2]
        # loose / transition
        lam=0.9, K=1.5, phi=2.0, vmax=20.0, vzmax=3.0,
        # strict / landing
        lam_land=1.6, K_land=0.6, phi_land=0.4, vmax_land=1.5, vzmax_land=0.6,
    )


class SMCOuter:
    def __init__(self, **kw):
        self.p = default_params()
        self.p.update({k: v for k, v in kw.items() if k in _KEYS})
        self.sp = None                       # converging set-point [x,y,z]
        self.vprev = np.zeros(3)             # last velocity cmd (for accel limiting)

    def set_params(self, **kw):
        self.p.update({k: float(v) for k, v in kw.items() if k in _KEYS})

    def update(self, p, target, dt):
        """p: vehicle position (3,), target: (3,), dt [s]. Returns (sp[3], v_cmd[3])."""
        p = np.asarray(p, dtype=float)
        target = np.asarray(target, dtype=float)
        # (re)seed the set-point at the vehicle on start OR after a big jump (restart / reset)
        if self.sp is None or np.linalg.norm(p - self.sp) > self.p["reset_dist"]:
            self.sp = p.copy()
        land = target[2] < self.p["land_z"]
        s = "_land" if land else ""
        lam = self.p["lam" + s]; K = self.p["K" + s]; phi = max(self.p["phi" + s], 1e-3)
        vmax = self.p["vmax" + s]; vzmax = self.p["vzmax" + s]
        e = self.sp - target
        v = -lam * e - K * np.clip(e / phi, -1.0, 1.0)          # sliding-mode velocity command
        vh = math.hypot(v[0], v[1])
        if vh > vmax and vh > 1e-9:
            v[0] *= vmax / vh; v[1] *= vmax / vh                 # horizontal speed limit
        v[2] = max(-vzmax, min(vzmax, v[2]))                     # climb/descent limit
        amax = self.p["amax"] * dt                               # accel-limit the velocity cmd (smooth
        v = np.clip(v, self.vprev - amax, self.vprev + amax)     # ramp -> no lurch on a far-target jump)
        self.vprev = v.copy()
        new = self.sp + v * dt
        for i in range(3):                                       # clamp: never overshoot target
            if (new[i] - target[i]) * (self.sp[i] - target[i]) < 0.0:
                new[i] = target[i]
        self.sp = new
        return self.sp.copy(), v
