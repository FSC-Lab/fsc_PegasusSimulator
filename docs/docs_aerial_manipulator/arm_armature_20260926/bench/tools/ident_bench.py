#!/usr/bin/env python3
"""Identify each servo UNIT's armature (reflected rotor + gear-train inertia) from the arm's
ground-bench calibration bags, 2026-09-09 and 2026-09-11.

    PYTHONNOUSERSITE=1 /usr/bin/python3 ident_bench.py [fc_hz]      (after extract_bench.py)

UNITS, by the ID each carried before the 2026-09-11 swap (the user's naming):
    session 0909  slots j1..j4 = 11, 12, 13, 14      (the "ID 12 & 13" folder)
    session 0911  slots j1..j4 = 12, 11, 14, 13      (the "ID 11 & 14" folder)
    FLIGHT        slots J1..J4 = 12, 11, 14, 13      (= session 0911)
so every unit is measured in TWO slots, one of them a gravity-loaded middle slot.

WHAT IS MEASURED. On a fixed base with ONE joint moving, that joint's own row is
    tau_j = (M_jj(q) + J_j) qdd_j + g_j(q) + friction_j(qdot_j)
with M_jj independent of q_j (serial chain) and hence CONSTANT through a single-joint sine,
and no Coriolis term on the moving joint's own row. tau_j = Present Current / kappa_unit.
So the TOTAL diagonal inertia M_tot = M_jj + J is measured model-free; the armature is
J = M_tot - M_jj,links with M_jj,links from the whole-body model (make_params_t650 with the
0.020 h h^T armature REMOVED), cross-checked in the report against the URDF.

Two estimators, both on the encoder (zero-phase low-pass, every signal through the same filter):
  A  REGRESSION over every single-joint window of a (session, slot):
        y = tau_meas - [M_links(q) qdd + C qdot + S g(q)]      (links-only model, all rows)
        y_j = J qdd_j + P_pose(q_j - c) + sum_window [c_w + fc_w tanh(qdot/w) + b_w qdot] + e
     P_pose is a CUBIC in the joint angle SHARED by every window at the same pose (absorbs the
     gravity-model error and any rate-independent stiffness); friction and offset are per window
     (Stribeck and thermal drift cannot then leak into J). J is identified only because the same
     pose is excited at different frequencies -- inertia scales with omega^2, P does not.
  B  HARMONIC, per sine window: in-phase (with q - c) coefficient k(omega) of y at the fundamental
     = K_rate_independent - J omega^2 ; pairs of windows at the same pose and different omega give
     J = -(k2 - k1)/(omega2^2 - omega1^2). Transparent, no friction model at all (any odd function
     of qdot and any rate-independent hysteresis project to quadrature or cancel in the difference).
Errors: leave-one-window-out jackknife (A) and the spread over repeated pairs (B) -- the formal
least-squares error is optimistic because the residual is coloured (backlash ripple, stick-slip).
"""
import copy, glob, os, sys
import numpy as np
from scipy.signal import butter, filtfilt

sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
FC = (float(sys.argv[1]) if __name__ == '__main__' and len(sys.argv) > 1 else 4.0)
FS = 250.0; DEC = 5                                  # analysis rate 50 Hz after the low-pass
P = TP.make_params_t650(); n = P["n"]
JA = 353.5 ** 2 * 1.6e-7
PL = copy.deepcopy(P)                                # links only: the model's h h^T armature removed
for i in range(n):
    h = np.asarray(P["h_i_im1"][i], float)
    PL["I_i_i"][i + 1] = np.asarray(P["I_i_i"][i + 1]) - JA * np.outer(h, h)
G_BENCH = np.array([0.0, 0.0, 9.81])                 # specific force on the fixed inverted mount (model base z up)
S_G = np.array([1.0, 0.897, 0.969, 1.0])             # gravity_scale follows the SLOT
KAPPA_UNIT = {11: 154.0, 12: 162.4, 13: 153.4, 14: 150.5}   # counts/N.m, follows the UNIT (flight yaml)
SLOT_UNIT = {"s0909": (11, 12, 13, 14), "s0911": (12, 11, 14, 13)}
FLIGHT_UNIT = (12, 11, 14, 13)
W_TANH = 0.015


def lp(x, fc):
    b, a = butter(4, fc / (FS / 2))
    return filtfilt(b, a, x, axis=0)


def windows(d, t):
    """single-joint windows: (joint, i0, i1, kind) on the uniform grid t; others held within 1 deg."""
    rv, rt = d["ref_v"], d["ref_t"]
    out = []
    for k in range(4):
        mv = np.abs(rv[:, k]) > np.radians(1.0)
        idx = np.where(mv)[0]
        if len(idx) < 50:
            continue
        for s in np.split(idx, np.where(np.diff(rt[idx]) > 3.0)[0] + 1):
            if len(s) < 100:
                continue
            a, b = rt[s[0]] - 0.5, rt[s[-1]] + 1.5          # include the settle after the move
            i0, i1 = np.searchsorted(t, a), np.searchsorted(t, b)
            if i1 - i0 < 100:
                continue
            others = [np.ptp(d["_q"][i0:i1, j]) for j in range(4) if j != k]
            if max(others) > np.radians(1.0):
                continue
            kind = "sine" if (b - a) > 15.0 else "move"
            out.append((k, i0, i1, kind))
    return out


def prepare(fn):
    d = dict(np.load(fn))
    th = d["js_t"]; t = np.arange(th[0], th[-1], 1.0 / FS)
    q = np.column_stack([np.interp(t, th, d["js_q"][:, k]) for k in range(4)])
    cur = np.column_stack([np.interp(t, th, d["js_i"][:, k]) for k in range(4)])
    d["_q"] = q
    wins = windows(d, t)
    qf = lp(q, FC); qd = np.gradient(qf, 1 / FS, axis=0); qdd = np.gradient(qd, 1 / FS, axis=0)
    sess = os.path.basename(fn).split("__")[0]
    kap = np.array([KAPPA_UNIT[u] for u in SLOT_UNIT[sess]])
    tau = lp(cur, FC) / kap
    keep = np.zeros(len(t), bool)
    for (_, i0, i1, _) in wins:
        keep[i0:i1] = True
    sel = np.where(keep)[0][::DEC]
    tl = np.full_like(q, np.nan); mjj = np.full_like(q, np.nan)
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(9, order="F")
    for i in sel:
        X[12:12 + n] = qf[i]; X[18 + n:18 + 2 * n] = qd[i]
        dy = CT.dynamics(X, PL); M = dy["M"]; C = dy["C"]
        V = np.concatenate([np.zeros(6), qd[i]])
        tl[i] = M[6:, 6:] @ qdd[i] + (C @ V)[6:] + S_G * (M[6:, 0:3] @ G_BENCH)
        mjj[i] = np.diag(M[6:, 6:])
    return dict(name=os.path.basename(fn)[:-4], sess=sess, t=t - t[0], t0=t[0], q=qf, qd=qd, qdd=qdd,
                tau=tau, tl=tl, y=tau - tl, mjj=mjj, wins=wins, sel=sel)


def fit_A(rows, jack=True):
    """rows: list of (D, k, i0, i1, kind); returns J, jackknife se, M_links mean, stats."""
    def build(rr):
        pose_keys = {}; cols = []; ys = []; Wn = len(rr)
        blocks = []
        for w, (D, k, i0, i1, kind) in enumerate(rr):
            s = D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)]
            qj = D["q"][s, k]; others = np.delete(D["q"][s].mean(0), k)
            c = np.median(qj)
            key = (D["sess"], k, tuple(np.round(np.degrees(others)).astype(int)), int(round(np.degrees(c))))
            pk = pose_keys.setdefault(key, len(pose_keys))
            blocks.append((D, k, s, qj - c, pk, w))
        nP = len(pose_keys)
        ncol = 1 + 3 * nP + 3 * Wn
        for (D, k, s, dq, pk, w) in blocks:
            A = np.zeros((len(s), ncol))
            A[:, 0] = D["qdd"][s, k]
            for p in range(3):
                A[:, 1 + 3 * pk + p] = dq ** (p + 1)
            b0 = 1 + 3 * nP + 3 * w
            A[:, b0] = 1.0
            A[:, b0 + 1] = np.tanh(D["qd"][s, k] / W_TANH)
            A[:, b0 + 2] = D["qd"][s, k]
            cols.append(A); ys.append(D["y"][s, k])
        A = np.vstack(cols); y = np.concatenate(ys)
        return A, y
    A, y = build(rows)
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = y - A @ c
    J = c[0]
    se = np.nan
    if jack and len(rows) > 2:
        Js = []
        for w in range(len(rows)):
            rr = rows[:w] + rows[w + 1:]
            A2, y2 = build(rr)
            c2, *_ = np.linalg.lstsq(A2, y2, rcond=None)
            Js.append(c2[0])
        Js = np.array(Js); m = len(Js)
        se = np.sqrt((m - 1) / m * np.sum((Js - Js.mean()) ** 2))
    mj = np.nanmean(np.concatenate([D["mjj"][D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)], k]
                                    for (D, k, i0, i1, _) in rows]))
    qdd95 = np.percentile(np.abs(A[:, 0]), 95)
    return J, se, mj, len(y), r.std(), qdd95


def harmonic(D, k, i0, i1):
    """in-phase / quadrature of y (and of the MEASURED tau) at the window's fundamental."""
    s = D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)]
    t = D["t"][s]; qj = D["q"][s, k]; c = np.median(qj)
    # fundamental from the zero crossings of q - c
    x = qj - c; zc = np.where(np.diff(np.sign(x)) != 0)[0]
    if len(zc) < 6:
        return None
    per = 2 * np.median(np.diff(t[zc]))
    om = 2 * np.pi / per
    # middle of the window only: skip the amplitude ramps
    ts = t[zc[1]]; te = t[zc[-2]]
    m = (t >= ts) & (t <= te)
    C_, S_ = np.cos(om * t[m]), np.sin(om * t[m])
    B = np.column_stack([C_, S_, np.ones(m.sum())])
    cq, *_ = np.linalg.lstsq(B, x[m], rcond=None)
    amp = np.hypot(cq[0], cq[1]); ph = np.arctan2(cq[1], cq[0])
    u_in = np.cos(om * t[m] - ph)                   # unit sinusoid in phase with q - c
    u_qd = np.sin(om * t[m] - ph)
    out = {}
    for nm, sig in (("y", D["y"][s, k][m]), ("tau", D["tau"][s, k][m]), ("qdd", D["qdd"][s, k][m])):
        Bb = np.column_stack([u_in, u_qd, np.ones(m.sum())])
        cc, *_ = np.linalg.lstsq(Bb, sig, rcond=None)
        out[nm] = (cc[0] / amp, cc[1] / amp)       # per rad of excursion
    ncyc = (te - ts) / per
    return dict(om=om, amp=amp, c=c, k_in=out["y"][0], k_qd=out["y"][1], ktau_in=out["tau"][0],
                qdd_gain=-out["qdd"][0], ncyc=ncyc)


def prepare_all(fc):
    global FC
    FC = fc
    cache = os.path.join(HERE, "..", f"prepared_fc{fc:g}.npy")
    if os.path.exists(cache):
        Ds = list(np.load(cache, allow_pickle=True))
        if all("t0" in D for D in Ds):
            return Ds
    # the sine / move analysis uses the eval and coupling bags only (the lever sweeps have their
    # own segmentation in calib_lever.py -- their merged passes would look like "sine" windows here)
    Ds = [prepare(f) for f in sorted(glob.glob(os.path.join(DATA, "*.npz")))
          if "__eval_" in f or "__coupling" in f]
    np.save(cache, np.array(Ds, dtype=object), allow_pickle=True)
    return Ds


if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(DATA, "*.npz")))
    Ds = []
    for f in files:
        D = prepare(f)
        Ds.append(D)
        print(f"prepared {D['name']}: {len(D['wins'])} single-joint windows", flush=True)
    np.save(os.path.join(HERE, "..", f"prepared_fc{FC:g}.npy"), np.array(Ds, dtype=object), allow_pickle=True)

    print(f"\n==== A: regression, low-pass {FC:g} Hz ====")
    resA = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            unit = SLOT_UNIT[sess][k]
            for label, kinds in (("sines", ("sine",)), ("sines+moves", ("sine", "move"))):
                rows = [(D, kk, i0, i1, kd) for D in Ds if D["sess"] == sess
                        for (kk, i0, i1, kd) in D["wins"] if kk == k and kd in kinds]
                if len(rows) < 2:
                    continue
                J, se, mj, N, rs, q95 = fit_A(rows)
                resA[(sess, k, label)] = (J, se, mj)
                print(f"  {sess} slot j{k+1} unit {unit}  [{label:11s}] windows {len(rows):2d}  N {N:6d}  "
                      f"|qdd|95 {q95:.2f}  J = {J:+.4f} +/- {se:.4f}  (M_links {mj:.4f} -> M_tot {J+mj:.4f})  "
                      f"resid {rs*1e3:.1f} mN.m")

    print(f"\n==== B: harmonic two-frequency pairs, low-pass {FC:g} Hz ====")
    resB = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            unit = SLOT_UNIT[sess][k]
            H = []
            for D in Ds:
                if D["sess"] != sess:
                    continue
                for (kk, i0, i1, kd) in D["wins"]:
                    if kk == k and kd == "sine":
                        h = harmonic(D, k, i0, i1)
                        if h:
                            h["bag"] = D["name"].split("__")[1]; h["mj"] = np.nanmean(D["mjj"][D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)], k])
                            H.append(h)
            if not H:
                continue
            print(f"  {sess} slot j{k+1} unit {unit}:")
            for h in H:
                print(f"     {h['bag']:22s} c {np.degrees(h['c']):5.1f} A {np.degrees(h['amp']):4.1f} deg  f {h['om']/2/np.pi:.4f} Hz "
                      f"({h['ncyc']:.1f} cyc)  k_in(y) {h['k_in']*1e3:+7.2f}  k_in(tau) {h['ktau_in']*1e3:+8.2f}  "
                      f"k_quad {h['k_qd']*1e3:+7.2f} mN.m/rad   qdd/q {h['qdd_gain']:.3f} (om^2 {h['om']**2:.3f})")
            # all pairs at the same centre (within 2 deg) with omega ratio > 1.4
            Js = []
            for a in range(len(H)):
                for b in range(len(H)):
                    ha, hb = H[a], H[b]
                    if hb["om"] / ha["om"] < 1.4 or abs(ha["c"] - hb["c"]) > np.radians(2.0):
                        continue
                    Jt = -(hb["k_in"] - ha["k_in"]) / (hb["qdd_gain"] - ha["qdd_gain"])
                    Js.append((ha["bag"], hb["bag"], Jt))
            if Js:
                v = np.array([x[2] for x in Js])
                resB[(sess, k)] = (np.median(v), v.std(), len(v))
                print(f"     pairs {len(v)}: J = median {np.median(v):+.4f}, mean {v.mean():+.4f}, std {v.std():.4f}  "
                      f"[{', '.join(f'{x[2]:+.4f}' for x in Js[:8])}{' ...' if len(Js) > 8 else ''}]")

    print("\n==== per UNIT, in FLIGHT slot order (J1..J4 = units 12, 11, 14, 13) ====")
    for J_, u in enumerate(FLIGHT_UNIT):
        line = f"  flight J{J_+1} = unit {u}: "
        for sess in ("s0909", "s0911"):
            k = SLOT_UNIT[sess].index(u)
            a = resA.get((sess, k, "sines"))
            b = resB.get((sess, k))
            line += (f"| {sess} slot j{k+1}: A {a[0]:+.4f}+/-{a[1]:.4f}" if a else f"| {sess} slot j{k+1}: A --") + \
                    (f", B {b[0]:+.4f} (sd {b[1]:.4f}, {b[2]} pairs) " if b else ", B -- ")
        print(line)
