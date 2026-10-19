"""The same question in SIMULATION: how does the basket hang on the claw in the Isaac pick-and-place runs?

Reads the mission recordings of the 2026-10-08/09 campaign (../../pick_place_top_hat_20261008/runs/*.npz: the true
payload pose `payload` = [t, p(3), q(wxyz)], the true claw point `claw` = [t, p(3)], `odom` = [t, p, v, q(wxyz)],
`joints` = [t, q(4), qd(4)], mission marks) and gives, in the claw's own vertical plane as tools/hang_follow.py does for
the flights: the fingers' pitch, the basket's tilt (top toward the fingertips +), and the basket centre against the
true claw point along the heading. Writes ../analysis/sim_hang.json.
usage: sim_hang.py [run tags ...]
"""
import json, sys
from pp10_common import *
from scipy.spatial.transform import Rotation as Rot

RUNS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "pick_place_top_hat_20261008", "runs"))
Rz = lambda a: np.array([[np.cos(a), -np.sin(a), 0.0], [np.sin(a), np.cos(a), 0.0], [0.0, 0.0, 1.0]])
Rq = lambda q: Rot.from_quat(np.asarray(q)[..., [1, 2, 3, 0]]).as_matrix()          # (w, x, y, z) -> matrix


def run(tag, table=False):
    d = np.load(os.path.join(RUNS_DIR, tag + ".npz"), allow_pickle=True)
    mk = dict(zip([str(x) for x in d["marks_name"]], [float(x) for x in d["marks_t"]]))
    od, pl, cl, jn = d["odom"], d["payload"], d["claw"], d["joints"]
    a, b = mk["exit_pick:end"] - 1.0, mk["exit_place:start"]
    tu = np.arange(a, b, 0.05)
    ip = lambda src, cols: np.column_stack([np.interp(tu, src[:, 0], src[:, c]) for c in cols])
    Pv = ip(od, (1, 2, 3)); Qv = ip(od, (7, 8, 9, 10)); Qv /= np.linalg.norm(Qv, axis=1)[:, None]
    Pb = ip(pl, (1, 2, 3)); Qb = ip(pl, (4, 5, 6, 7)); Qb /= np.linalg.norm(Qb, axis=1)[:, None]
    Pc = ip(cl, (1, 2, 3)); q = ip(jn, (1, 2, 3, 4))
    Rv, Rb = Rq(Qv), Rq(Qb)
    yawv = np.unwrap(np.arctan2(Rv[:, 1, 0], Rv[:, 0, 0]))
    pit = np.zeros(len(tu)); az = np.zeros(len(tu)); fk_err = np.zeros(len(tu))
    side = np.zeros(len(tu)); roll = np.zeros(len(tu)); rel = np.zeros((len(tu), 3))
    for k in range(len(tu)):
        Rm = Rv[k] @ RZM90; r0e, _, Re = TP.arm_fk_model(q[k], PARAMS)[:3] if False else (None, None, TP.arm_fk_model(q[k], PARAMS)[2])
        f = Rm @ (-Re[:, 2]); pit[k] = np.degrees(np.arctan2(-f[2], np.hypot(f[0], f[1]))); az[k] = np.arctan2(f[1], f[0])
        zc = Rz(-az[k]) @ Rb[k][:, 2]
        side[k] = np.degrees(np.arctan2(zc[0], zc[2])); roll[k] = np.degrees(np.arctan2(zc[1], zc[2]))
        rel[k] = 1e3 * (Rz(-yawv[k]) @ (Pb[k] - Pc[k]))              # basket centre from the true claw point: along the heading, left, up
    # touchdown: the basket's height stops falling in the place descent
    zf = float(d["payload_final"][2]); m = (tu > mk["place:start"]) & (Pb[:, 2] < zf + 0.004); t_td = float(tu[m][0]) if m.any() else b
    seg = lambda t0, t1: {k: float(v[(tu >= t0) & (tu <= t1)].mean()) for k, v in dict(pitch=pit, side=side, roll=roll, along=rel[:, 0], left=rel[:, 1], up=rel[:, 2], q1=np.degrees(q[:, 0]), q2=np.degrees(q[:, 1]), q3=np.degrees(q[:, 2])).items()}
    st = dict(after_lift=seg(mk["exit_pick:end"] - 0.6, mk["exit_pick:end"]),
              carry=seg(mk["go_to_place_start:start"] + 4.0, mk["go_to_place_start:end"] - 2.0),
              place_start=seg(mk["go_to_place_start:end"], mk["ready_place:start"]),
              above_place=seg(mk["ready_place:end"] - 1.0, mk["ready_place:end"]),
              descent=seg(t_td - 1.2, t_td - 0.1))
    n = int(2.0 / 0.05); ker = np.ones(n) / n; lo = lambda x: np.convolve(x, ker, mode="same")
    mm = (tu > mk["exit_pick:end"]) & (tu < t_td - 0.1); sf = (side - lo(side))[mm][n:-n]; pf = (pit - lo(pit))[mm][n:-n]
    fast = (float(np.polyfit(pf, sf, 1)[0]), float(np.corrcoef(pf, sf)[0, 1]))
    whole = (float(np.polyfit(pit[mm], side[mm], 1)[0]), float(np.corrcoef(pit[mm], side[mm])[0, 1]))
    out = dict(tag=tag, rig=str(d["rig"]), touchdown=t_td, stages=st, fast_slope=fast, whole_slope=whole,
               side_range=[float(side[mm].min()), float(side[mm].max())], pitch_range=[float(pit[mm].min()), float(pit[mm].max())],
               t=tu[::4].round(2).tolist(), pitch_s=pit[::4].round(2).tolist(), side_s=side[::4].round(2).tolist(), along_s=rel[::4, 0].round(1).tolist())
    print(f"== {tag} ({out['rig']}): lift ends {mk['exit_pick:end']:.1f}, touchdown {t_td:.1f}; pitch {out['pitch_range'][0]:.1f}..{out['pitch_range'][1]:.1f}, basket tilt {out['side_range'][0]:.1f}..{out['side_range'][1]:.1f}")
    print(f"   tilt on pitch: whole slope {whole[0]:+.2f} (corr {whole[1]:+.2f}); faster than 2 s slope {fast[0]:+.2f} (corr {fast[1]:+.2f})")
    for k, s in st.items():
        print(f"   {k:12s} pitch {s['pitch']:5.1f} | tilt {s['side']:+5.1f} roll {s['roll']:+5.1f} | centre along {s['along']:+6.1f} left {s['left']:+6.1f} up {s['up']:+7.1f} | q {s['q1']:5.1f} {s['q2']:5.1f} {s['q3']:5.1f}")
    if table:
        for k in range(0, len(tu), 20):
            print(f"   {tu[k]:6.1f}  pitch {pit[k]:5.1f}  tilt {side[k]:+5.1f}  roll {roll[k]:+5.1f}  along {rel[k, 0]:+6.1f}  up {rel[k, 2]:+7.1f}")
    return out


if __name__ == "__main__":
    tags = sys.argv[1:] or ["cmp_wb_r1_a1", "cmp_wb_r2_a2", "hat1", "cmp_geo_r1_a1", "cmp_geo_r2_a1"]
    res = {}
    for tg in tags:
        try:
            res[tg] = run(tg, table=(tg == tags[0]))
        except Exception as e:                                   # a run without the marks (an aborted attempt)
            print(f"== {tg}: skipped ({type(e).__name__}: {e})")
    json.dump(res, open(os.path.join(OUT10, "sim_hang.json"), "w"), indent=1)
    print("wrote sim_hang.json")
