#!/usr/bin/env python3
"""Does the valve's rotor LEAN when the clamped claw moves?  (user, 2026-10-29: the printed shaft is
not stiff, so the fin may roll when the arm pushes it.)

For each 10-28 flight, on a 50 Hz grid:
  * obj_0's orientation relative to the claw's arrival on the handle, with the pure rotation about the
    fitted shaft removed; the residual LEAN split in the valve's polar frame at the fin:
      roll  = about the tangent (= about the vehicle's nose): + = fin top leans radially OUTWARD
      pitch = about the radius: + = fin top leans along +tangent (the turn direction)
  * an INDEPENDENT check of the roll from obj_0's POSITION: the marker centroid sits ~73 mm out from
    the shaft, so a roll alpha lowers it by ~0.073*alpha (vertical) -- mocap position is far less noisy
    than its orientation;
  * the claw's own roll about the tangent (planner FK: measured joints + PX4 attitude);
  * the vehicle's sideways (radial) CoM error and the joints q1/q4.
Everything band-passed to the sway band (0.35-0.9 Hz).  Noise floor = the same quantities with the
vehicle hovering away from the valve before Go To Start."""
import sys
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times, rotvec, quat_R

AXIS = {"145526": (0.855, 0.004)}   # the planner's captured axis
BAND = (0.35, 0.9)
FS = 50.0


def rot(axis, ang):
    """Rodrigues, one axis, many angles."""
    k = axis / np.linalg.norm(axis); K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    s, c = np.sin(ang)[:, None, None], np.cos(ang)[:, None, None]
    return np.eye(3)[None] + s * K[None] + (1 - c) * (K @ K)[None]


def residual_lean(R, R0, s):
    """World-frame residual rotation vector (rad) after the rotation about shaft s:  R R0^T = Res Rot(s, th)."""
    Rrel = R @ R0.T
    th, a = rotvec(Rrel); th = th * np.sign(a @ s)          # first guess: the whole rotation about s
    for _ in range(3):
        Res = Rrel @ np.transpose(rot(s, th), (0, 2, 1))
        dth, da = rotvec(Res)
        th = th + dth * (da @ s)
    Res = Rrel @ np.transpose(rot(s, th), (0, 2, 1))
    dth, da = rotvec(Res)
    return dth[:, None] * da, th


def bandpass(y, fs=FS, band=BAND):
    y = np.asarray(y, float); ok = np.isfinite(y)
    if ok.sum() < 10:
        return np.full_like(y, np.nan)
    yy = np.interp(np.arange(len(y)), np.flatnonzero(ok), y[ok])
    yy = yy - np.polyval(np.polyfit(np.arange(len(yy)), yy, 1), np.arange(len(yy)))
    F = np.fft.rfft(yy); f = np.fft.rfftfreq(len(yy), 1 / fs)
    F[(f < band[0]) | (f > band[1])] = 0
    out = np.fft.irfft(F, len(yy)); out[~ok] = np.nan
    return out


def stats(w, *ys):
    return [np.sqrt(np.nanmean(y[w] ** 2)) for y in ys]


def corr(w, a, b):
    k = w & np.isfinite(a) & np.isfinite(b)
    return np.corrcoef(a[k], b[k])[0, 1]


def slope(w, y, x):
    k = w & np.isfinite(x) & np.isfinite(y)
    return np.sum(x[k] * y[k]) / np.sum(x[k] ** 2)


for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag); st = step_times(b); ax = np.array(AXIS[tag])
    A = np.load(f"../npz/axis_{tag}.npz"); T = np.load(f"../npz/turn_{tag}.npz")
    to, R, p = obj(b)
    i0 = np.searchsorted(to, st["handle"]); R0 = R[i0]
    s = R0 @ A["ab"]; s = s * np.sign(s[2]); s /= np.linalg.norm(s)
    lean_w, th = residual_lean(R, R0, s)
    G = np.arange(max(to[0], 2.0), to[-1] - 0.5, 1 / FS)
    # polar frame at the fin: the claw's measured azimuth about the captured axis (fixed before the handle)
    te = b.t("pl_current_ee")
    ee = np.stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"], 1)
    phi_e = np.unwrap(np.arctan2(ee[:, 1] - ax[1], ee[:, 0] - ax[0]))
    phi = np.interp(G, te, phi_e); phi[G < st["handle"]] = np.interp(st["handle"], te, phi_e)
    tang = np.stack([-np.sin(phi), np.cos(phi), 0 * phi], 1); radl = np.stack([np.cos(phi), np.sin(phi), 0 * phi], 1)
    Lw = np.stack([np.interp(G, to, lean_w[:, i]) for i in range(3)], 1)
    roll = np.degrees(np.sum(Lw * tang, 1)); pitch = -np.degrees(np.sum(Lw * radl, 1))
    # centroid height (position check of the roll)
    zc = np.interp(G, to, p[:, 2]) * 1e3
    rho = np.interp(G, to, np.hypot(p[:, 0] - ax[0], p[:, 1] - ax[1])) * 1e3
    # claw roll about the tangent (FK orientation), same decomposition
    qe = [b.f("pl_current_ee", f"pose.orientation.{c}") for c in "xyzw"]
    Re = quat_R(*qe); j0 = np.searchsorted(te, st["handle"])
    lean_e, _ = residual_lean(Re, Re[j0], s)
    Le = np.stack([np.interp(G, te, lean_e[:, i]) for i in range(3)], 1)
    claw_roll = np.degrees(np.sum(Le * tang, 1))
    # vehicle sideways error and joints (turn npz covers handle-2 .. abort+4)
    def from_turn(y):
        out = np.interp(G, T["g"], y); out[(G < T["g"][0]) | (G > T["g"][-1])] = np.nan; return out
    ecr = from_turn(T["ec_r"]) * 1e3
    q1 = from_turn(T["q"][:, 0]); q4 = from_turn(T["q"][:, 3])

    bp = {k: bandpass(v) for k, v in dict(roll=roll, pitch=pitch, zc=zc, rho=rho, claw=claw_roll,
                                          ecr=ecr, q1=q1, q4=q4).items()}
    t_abort = st.get("abort", G[-1])
    W = {"free, far from the valve": (G > 6) & (G < st["handle"] - 21),
         "Ready descent (claw at the fin, open)": (G > st["handle"] - 3.0) & (G < st["handle"]),
         "CLAMPED (handle -> abort)": (G > st["handle"]) & (G < t_abort)}
    print(f"===== {tag}: handle {st['handle']:.2f}  push {st['push']:.2f}  abort {t_abort:.2f}  "
          f"shaft {np.degrees(np.arccos(s[2])):.2f} deg off vertical; band {BAND[0]}-{BAND[1]} Hz rms")
    print(f"  {'window':40s} {'s':>5s} | roll  pitch (deg) | centroid z  rho (mm) | claw roll (deg) | sideways CoM (mm) | q1  q4 (deg)")
    for name, w in W.items():
        r, pt, z, rh, cr, e, a1, a4 = stats(w, bp["roll"], bp["pitch"], bp["zc"], bp["rho"], bp["claw"], bp["ecr"], bp["q1"], bp["q4"])
        print(f"  {name:40s} {w.sum()/FS:5.1f} | {r:5.2f} {pt:6.2f}       | {z:7.2f} {rh:7.2f}      | {cr:7.2f}         | {e:7.1f}           | {a1:4.1f} {a4:4.1f}")
    w = W["CLAMPED (handle -> abort)"]
    print(f"  clamped: corr(roll, centroid z) {corr(w, bp['roll'], bp['zc']):+.2f}, slope {slope(w, bp['zc'], bp['roll']):+.2f} mm/deg "
          f"(a pure lean at the centroid's {np.nanmedian(rho[w]):.0f} mm radius: -{np.nanmedian(rho[w])*np.pi/180:.2f})")
    print(f"           corr(roll, sideways CoM) {corr(w, bp['roll'], bp['ecr']):+.2f}, slope {slope(w, bp['roll'], bp['ecr']):+.4f} deg/mm;"
          f"  corr(roll, claw roll) {corr(w, bp['roll'], bp['claw']):+.2f}, ratio fin/claw {slope(w, bp['roll'], bp['claw']):+.2f}")
    print(f"           corr(claw roll, sideways CoM) {corr(w, bp['claw'], bp['ecr']):+.2f};  corr(roll, q1) {corr(w, bp['roll'], bp['q1']):+.2f}, "
          f"corr(roll, q4) {corr(w, bp['roll'], bp['q4']):+.2f};  corr(pitch, sideways CoM) {corr(w, bp['pitch'], bp['ecr']):+.2f}")
    # growth of the roll through the clamped window, in 2 s bins
    edges_ = np.arange(st["handle"], t_abort + 1e-6, 2.0)
    row = []
    for a0, a1 in zip(edges_[:-1], edges_[1:]):
        k = (G >= a0) & (G < a1)
        row.append(f"{a0 - st['handle']:4.0f}-{a1 - st['handle']:.0f}s: roll {np.sqrt(np.nanmean(bp['roll'][k]**2)):.2f} / "
                   f"z {np.sqrt(np.nanmean(bp['zc'][k]**2)):.2f} mm / claw {np.sqrt(np.nanmean(bp['claw'][k]**2)):.2f} / CoM {np.sqrt(np.nanmean(bp['ecr'][k]**2)):.0f} mm")
    print("  2 s bins after the handle:\n    " + "\n    ".join(row))

    # what drives the roll: a radial PUSH (task spring K_y * radial EE error, through the grasp height)
    # or a roll COUPLE from the claw (the wrist-roll joint q4's torque, its axis ~ along the tangent)?
    KY, LEVER = 120.0, 0.17        # N/m (valve yaml), grasp height above the bearing (m)
    phg = np.interp(T["g"], G, phi)
    eyr = np.sum(T["ey"][:, :2] * np.stack([np.cos(phg), np.sin(phg)], 1), 1)
    Fr = bandpass(from_turn(KY * eyr))
    tau4 = bandpass(from_turn(T["tau"][:, 3]))
    def kfit(y):
        k_ = w & np.isfinite(y) & np.isfinite(bp["roll"])
        return np.sum(y[k_] * np.radians(bp["roll"][k_])) / np.sum(np.radians(bp["roll"][k_]) ** 2)
    print(f"  drive: radial push {np.sqrt(np.nanmean(Fr[w]**2)):.2f} N rms, corr(push, roll) {corr(w, Fr, bp['roll']):+.2f};  "
          f"wrist-roll torque {np.sqrt(np.nanmean(tau4[w]**2))*1e3:.0f} mN.m rms, corr(tau4, roll) {corr(w, tau4, bp['roll']):+.2f} "
          f"-> roll stiffness ~ {abs(kfit(tau4)):.1f} N.m/rad ({abs(kfit(tau4))*np.pi/180*1e3:.0f} mN.m per deg)")
