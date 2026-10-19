"""What sets the angle the basket hangs at: does it pivot freely on the arch, or does it turn with the claw?

For the loaded part of PP-2 (10-09) and PP-4 (10-10), in the frame of the CLAW's azimuth (so the 12 deg arm yaw of the
carry pose does not mix the axes):
  pitch  the fingers' pitch below the horizontal (wrist -z through the planner FK on the vehicle's mocap attitude)
  side   the basket's tilt about the claw's lateral axis, top toward the fingertips (+), rest reading taken out
  roll   its tilt about the fingers' axis
A basket pivoting freely on the arch keeps side = atan(27.5 / h) = 9 deg whatever the claw does (slope 0);
a basket held by the closed jaws turns with the claw (slope 1).
Prints the regression of side on pitch (whole loaded window, and the fast part alone) and a 1 s table.
Writes ../analysis/hang_follow.json.
"""
import json
from pp10_common import *
import grasp_geom as GG
from scipy.spatial.transform import Rotation as Rot

WIN = {"p2": (50.5, 89.3, (2.0, 24.0)), "p4": (34.6, 72.6, (2.0, 16.0))}       # loaded window, rest window [s]
Rz = lambda a: np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])


def run(nm, table=False):
    S = GG.series(nm); d, t0, t = S["d"], S["t0"], S["t"]
    tb, Pb, Qb, _ = mocap_pose(d, t0, "obj")
    Rb = Rot.from_quat(Qb[:, [1, 2, 3, 0]]).as_matrix(); yawb = yaw_of(Qb)
    a, b, (ra, rb) = WIN[nm]
    m = (tb > ra) & (tb < rb) & (np.abs(np.degrees(yawb)) < 90); k0 = np.where(m)[0][len(np.where(m)[0]) // 2]
    R_off = Rz(-yawb[k0]) @ Rb[k0]
    # the claw on the 50 Hz grid
    mt = (t >= a) & (t <= b); idx = np.where(mt)[0]
    pit = np.zeros(len(idx)); az = np.zeros(len(idx))
    for j, k in enumerate(idx):
        Rm = quat_R(*S["Qv"][k]) @ RZM90; _, _, Re = TP.arm_fk_model(S["q"][k], PARAMS)
        f = Rm @ (-Re[:, 2]); pit[j] = np.degrees(np.arctan2(-f[2], np.hypot(f[0], f[1]))); az[j] = np.arctan2(f[1], f[0])
    tg = t[idx]; az = np.unwrap(az)
    # the basket at its own samples
    mb = (tb >= a) & (tb <= b); kb = np.where(mb)[0]
    azb = np.interp(tb[kb], tg, az); pb = np.interp(tb[kb], tg, pit)
    side = np.zeros(len(kb)); roll = np.zeros(len(kb)); ok = np.zeros(len(kb), bool)
    for j, k in enumerate(kb):
        Rc = Rb[k] @ R_off.T; zc = Rz(-azb[j]) @ Rc[:, 2]
        side[j] = np.degrees(np.arctan2(zc[0], zc[2])); roll[j] = np.degrees(np.arctan2(zc[1], zc[2]))
        ok[j] = abs(np.degrees(wrap(yawb[k] - azb[j]))) < 60 and abs(side[j]) < 60 and abs(roll[j]) < 60     # a flipped orientation sample
    tt, side, roll, pb = tb[kb][ok], side[ok], roll[ok], pb[ok]
    # onto a uniform 20 Hz grid for the filters
    tu = np.arange(tt[0], tt[-1], 0.05); su = np.interp(tu, tt, side); pu = np.interp(tu, tt, pb); ru = np.interp(tu, tt, roll)
    q = np.degrees(np.array([np.interp(tu, t, S["q"][:, i]) for i in range(4)]).T)
    fit = lambda x, y: (float(np.polyfit(x, y, 1)[0]), float(np.polyfit(x, y, 1)[1]), float(np.corrcoef(x, y)[0, 1]))
    n = int(2.0 / 0.05); ker = np.ones(n) / n
    lo = lambda x: np.convolve(x, ker, mode="same")
    sl, pl = lo(su), lo(pu); sf, pf = (su - sl)[n:-n], (pu - pl)[n:-n]
    # events: the arm leaves the pick hover (q2 up 3 deg), joint 2's largest jump in the carry, the arm back from its 12 deg yaw
    i_leave = int(np.argmax(q[:, 1] > q[:40, 1].mean() + 3.0)); i_yaw = int(np.argmax(q[:, 0] > 10.0))
    back = np.where((np.arange(len(tu)) > i_yaw) & (q[:, 0] < 6.0))[0]; i_back = int(back[0]) if len(back) else len(tu) - 1
    w = int(0.8 / 0.05); dq = np.abs(q[w:, 1] - q[:-w, 1]); dq[:i_yaw] = 0; dq[i_back:] = 0; i_jump = int(np.argmax(dq))
    seg = lambda i0, i1: (float(pu[i0:i1].mean()), float(su[i0:i1].mean()))
    carry = seg(i_jump + w + 10, i_back - 5); place = seg(len(tu) - 40, len(tu) - 5); lift = seg(5, max(i_leave - 5, 25))
    ev = dict(leave=float(tu[i_leave]), jump=float(tu[i_jump + w // 2]), jump_deg=float(dq[i_jump]), back=float(tu[i_back]), touchdown=float(b))
    out = dict(run=RUNS[nm][0], window=[a, b], unflipped=float(ok.mean()), ev=ev, lift=lift, carry=carry, place=place,
               follow=float((place[1] - carry[1]) / (place[0] - carry[0])),
               all=dict(zip(("slope", "offset", "corr"), fit(pu, su))), slow=dict(zip(("slope", "offset", "corr"), fit(pl[n:-n], sl[n:-n]))),
               fast=dict(zip(("slope", "offset", "corr"), fit(pf, sf)), pitch_rms=float(pf.std()), side_rms=float(sf.std())),
               pitch=[float(pu.min()), float(pu.mean()), float(pu.max())], side=[float(su.min()), float(su.mean()), float(su.max())],
               diff=dict(mean=float((su - pu).mean()), std=float((su - pu).std())), free_hang=float(np.degrees(np.arctan2(27.5, {"p2": 163.0, "p4": 170.8}[nm]))),
               t=tu[::4].round(2).tolist(), pitch_s=pu[::4].round(2).tolist(), side_s=su[::4].round(2).tolist(), roll_s=ru[::4].round(2).tolist(),
               q2_s=q[::4, 1].round(1).tolist(), q1_s=q[::4, 0].round(1).tolist())
    print(f"== {out['run']}: loaded {a}-{b} s, unflipped {100 * out['unflipped']:.0f} %, free hang would be {out['free_hang']:.1f} deg")
    print(f"   claw pitch {pu.min():.1f} .. {pu.max():.1f} (mean {pu.mean():.1f}); basket side tilt {su.min():.1f} .. {su.max():.1f} (mean {su.mean():.1f}); side - pitch {out['diff']['mean']:+.1f} +- {out['diff']['std']:.1f}")
    print(f"   events {ev}\n   (pitch, side): after the lift {lift[0]:.1f}, {lift[1]:.1f} | carry after the jump {carry[0]:.1f}, {carry[1]:.1f} | before the touchdown {place[0]:.1f}, {place[1]:.1f} | followed {100 * out['follow']:.0f} %")
    for kx in ("all", "slow", "fast"):
        print(f"   side = {out[kx]['offset']:+5.1f} + {out[kx]['slope']:.2f} x pitch   corr {out[kx]['corr']:.2f}   ({kx})")
    print(f"   fast part (under 2 s): pitch {out['fast']['pitch_rms']:.2f} deg rms, side {out['fast']['side_rms']:.2f} deg rms")
    if table:
        print("      t    pitch   side   roll  side-pitch   q1     q2")
        for i in range(0, len(tu), 20):
            print(f"   {tu[i]:6.1f}  {pu[i]:6.1f} {su[i]:6.1f} {ru[i]:6.1f}   {su[i] - pu[i]:+6.1f}   {q[i, 0]:6.1f} {q[i, 1]:6.1f}")
    return out


if __name__ == "__main__":
    res = {nm: run(nm) for nm in ("p2", "p4")}
    json.dump(res, open(os.path.join(OUT10, "hang_follow.json"), "w"), indent=1)
    print("wrote hang_follow.json")
