#!/usr/bin/env python3
"""Was the handle-tilt guard's reading a real tilt?  For each flight: the turn's step times, obj_0's
z-axis tilt against its pose when the claw first sat on the handle (the guard's own measure,
checked against push_pull/arrival_error [12]), the rotation axis fitted from obj_0's orientation
over the turn, gamma = the angle between obj_0's z axis and that axis, and the tilt a pure rotation
by theta would read: 2 asin(sin gamma sin(theta/2))."""
import sys
import numpy as np
from vt_common import Bag, BAGS

def quat_R(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w); x, y, z, w = x/n, y/n, z/n, w/n
    R = np.empty((len(x), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-z*w); R[:, 0, 2] = 2*(x*z+y*w)
    R[:, 1, 0] = 2*(x*y+z*w); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-x*w)
    R[:, 2, 0] = 2*(x*z-y*w); R[:, 2, 1] = 2*(y*z+x*w); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R

def rotvec(R):
    c = np.clip((np.trace(R, axis1=1, axis2=2)-1)/2, -1, 1); th = np.arccos(c)
    v = np.stack([R[:, 2, 1]-R[:, 1, 2], R[:, 0, 2]-R[:, 2, 0], R[:, 1, 0]-R[:, 0, 1]], 1)
    s = np.linalg.norm(v, axis=1); a = v / np.where(s > 1e-12, s, 1)[:, None]
    return th, a

def step_times(b):
    ed = b.edges("pl_pstatus")
    out = {}
    for t, s in ed:
        if s.startswith("DONE ready") and "handle" not in out: out["handle"] = t
        if s.startswith("FLYING push") and "push" not in out: out["push"] = t
        if s.startswith("ABORTING") and "abort" not in out: out["abort"] = t; out["abort_msg"] = s
        if s.startswith("DONE push") and "push_end" not in out: out["push_end"] = t
    return out

def obj(b, key="obj"):
    t = b.t(key)
    q = [b.f(key, f"pose.orientation.{c}") for c in "xyzw"]
    p = np.stack([b.f(key, f"pose.position.{c}") for c in "xyz"], 1)
    return t, quat_R(*q), p

if __name__ == "__main__":
    for tag in (sys.argv[1:] or BAGS):
        b = Bag(tag); st = step_times(b)
        t, R, p = obj(b)
        th0 = st["push"] + 1.5
        print(f"===== {tag}: claw on the handle {st['handle']:.2f}  push step {st['push']:.2f}  turn start {th0:.2f}  "
              f"abort {st.get('abort', np.nan):.2f} ({st.get('abort', np.nan)-th0:.2f} s into the 18 s turn)")
        i0 = np.searchsorted(t, st["handle"])
        R0 = R[i0]
        tilt = np.degrees(np.arccos(np.clip(np.einsum('i,ni->n', R0[:, 2], R[:, :, 2]), -1, 1)))
        Rrel = np.einsum('ji,njk->nik', R0, R)          # R0^T R(t), body frame
        th, a = rotvec(Rrel)
        yaw = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
        ta = st.get("abort", t[-1]); w = (t >= th0) & (t <= ta + 0.5) & (np.degrees(th) > 8)
        # axis: the body-frame axis, sign-aligned to the mean, then the mean
        A = a[w]; A = A * np.sign(A @ A.mean(0))[:, None]; ab = A.mean(0); ab /= np.linalg.norm(ab)
        spread = np.degrees(np.arccos(np.clip(A @ ab, -1, 1)))
        gam = np.degrees(np.arccos(abs(ab[2])))
        aw = R0 @ ab   # in the world
        shaft_tilt = np.degrees(np.arccos(abs(aw[2])))
        print(f"  obj_0 samples {len(t)}  rate {1/np.median(np.diff(t)):.0f} Hz  max gap {np.diff(t).max()*1e3:.0f} ms")
        print(f"  rotation axis (obj_0 body frame) {np.round(ab, 4)}  spread {np.median(spread):.2f} deg (p95 {np.percentile(spread, 95):.2f})")
        print(f"  gamma = angle(obj_0 z, shaft) = {gam:.2f} deg ;  shaft vs world vertical = {shaft_tilt:.2f} deg")
        # residual tilt after removing the pure rotation about ab
        pred = 2*np.degrees(np.arcsin(np.sin(np.radians(gam))*np.sin(th/2)))
        for tt in (st["handle"], th0, th0 + 2, th0 + 4, th0 + 6, th0 + 8, ta - 0.3, ta, ta + 1, ta + 3):
            j = min(np.searchsorted(t, tt), len(t)-1)
            print(f"   t {t[j]:7.2f}  rot {np.degrees(th[j]):6.2f} deg  yaw {yaw[j]-yaw[i0]:+7.2f}  tilt {tilt[j]:5.2f}  "
                  f"pure-rotation tilt {pred[j]:5.2f}  axis-pt drift {np.linalg.norm((p[j]-p[i0])[:2])*1e3:6.1f} mm")
        if b.has("pl_arr"):
            ta_, ar = b.t("pl_arr"), b.f("pl_arr", "data")
            k = (ta_ > st["handle"]) & (ta_ <= ta + 0.05)
            j = np.searchsorted(t, ta_[k])
            print(f"  planner tilt [12] vs this tilt (same samples): max |diff| {np.nanmax(np.abs(ar[k, 12]-tilt[np.minimum(j, len(t)-1)])):.2f} deg, "
                  f"planner peak {np.nanmax(ar[k, 12]):.2f}")
        np.savez(f"../npz/axis_{tag}.npz", t=t, tilt=tilt, rot=np.degrees(th), yaw=yaw-yaw[i0], pred=pred, gamma=gam,
                 ab=ab, t_handle=st["handle"], t_push=st["push"], t_turn=th0, t_abort=st.get("abort", np.nan))
