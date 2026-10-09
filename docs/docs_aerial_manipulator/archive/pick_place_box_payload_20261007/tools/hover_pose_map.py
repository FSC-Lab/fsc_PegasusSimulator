#!/usr/bin/env python3
"""Which arm poses to test in hover before the hook pick-and-place (2026-10-07).

    PYTHONNOUSERSITE=1 /usr/bin/python3 hover_pose_map.py [--plot ../runs/hover_pose_map.png]

1. Where the singular regions are: sigma_nd (the planner's keep-out metric,
   sigma_min of J_3y^0 with the translational rows / Lchar, keep-out 0.10) over
   q2 x q3, and whether q1 / q4 change it.
2. Per candidate pose: sigma_nd (kinematic) and the law's own J_3y sigma, the
   margins to the real arm's stops (q2 -20..+45, q3 +50; q3 >= 0 keeps off the
   elbow branch), the J2 / J3 hold torque with and without the 200 g basket
   hung at the grasp point, and the STANDING MOMENT the pose puts on the
   airframe (system CoM offset from the arm-at-home CoM, x mg) -- the moment
   the observer / PX4 integrator has to re-learn on every pose change.
"""
import argparse

import numpy as np

from pose_sweep import P, N, D, CT, TP, g_arm, sig_law

MTOT = sum(P["m_i"])
G = 9.81
MPAY = 0.200


def claw(q):
    return CT._arm_kin(q, P)[1]


def hung_tau(q, m=MPAY):
    e = 1e-6
    J = np.array([(claw(q + e * np.eye(N)[j]) - claw(q - e * np.eye(N)[j])) / (2 * e) for j in range(N)]).T
    return g_arm(q) + J.T @ np.array([0.0, 0.0, m * G])


def com_offset(q, m_pay=0.0):
    """System CoM (base frame, MODEL axes) incl. an optional load at the grasp point."""
    r0c, r0e = CT._arm_kin(q, P)
    return (MTOT * r0c + m_pay * r0e) / (MTOT + m_pay)


HOME = D([0, 40, 40, 0])

# (label, pose deg)
CANDIDATES = [
    ("home (takeoff/landing)", [0, 40, 40, 0]),
    ("pick / place", [0, 32, 38, 0]),
    ("carry (place start)", [12, 38, 42, 0]),
    ("lift dip (q2 -12)", [0, 20, 35, 0]),
    ("set-down spring (+8/+7)", [0, 40, 45, 0]),
    ("carry, q1 -12", [-12, 38, 42, 0]),
    ("carry, q1 +20 (beyond)", [20, 38, 42, 0]),
    ("pick, wrist q4 +30", [0, 32, 38, 30]),
    ("pick, wrist q4 -30", [0, 32, 38, -30]),
    ("unfolded beta 50", [0, 25, 25, 0]),
    ("unfolded beta 30", [0, 15, 15, 0]),
    ("q3 near 0 (beta 20)", [0, 20, 0, 0]),
    ("beta 10 (claw near vertical)", [0, 0, 10, 0]),
    ("beta 0 (claw vertical)", [0, -10, 10, 0]),
    ("q3 -20 (toward the valley)", [0, 20, -20, 0]),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", default=None)
    a = ap.parse_args()

    # 1. q1 / q4 invariance
    base = TP._sigma_nd(D([0, 32, 38, 0]), P)
    v = [TP._sigma_nd(D([q1, 32, 38, q4]), P) for q1 in (-30, 0, 30) for q4 in (-90, 0, 90)]
    print(f"sigma_nd at q2/q3 = 32/38 over q1 in {{-30,0,30}} x q4 in {{-90,0,90}}: "
          f"{min(v):.4f} .. {max(v):.4f} (q1=q4=0: {base:.4f})")

    # 2. the map over q2 x q3
    q2s = np.arange(-20, 45.01, 1.0)
    q3s = np.arange(-40, 50.01, 1.0)
    S = np.array([[TP._sigma_nd(D([0, a2, a3, 0]), P) for a2 in q2s] for a3 in q3s])
    print("\nsigma_nd minima: global over the real q2 range "
          f"{S.min():.3f}; along beta = q2 + q3:")
    for b in (-10, 0, 5, 10, 20, 30, 50, 70, 80, 90):
        vals = [S[i, j] for i, a3 in enumerate(q3s) for j, a2 in enumerate(q2s) if abs(a2 + a3 - b) < 0.5]
        if vals:
            print(f"   beta {b:4d}: {min(vals):.3f} .. {max(vals):.3f}")
    print("   at q3 < 0 (elbow branch side), min "
          f"{S[q3s < 0].min():.3f}; at q3 >= 0, beta >= 30: "
          f"{min(S[i, j] for i, a3 in enumerate(q3s) for j, a2 in enumerate(q2s) if a3 >= 0 and a2 + a3 >= 30):.3f}")

    # 3. candidates
    c_home = com_offset(HOME)
    print(f"\n{'pose':29s} {'q (deg)':18s} beta  sig_nd sig_law  q2-(-20) 45-q2 50-q3  "
          f"|t2| |t3| hold  |t2| |t3| +200g  dM_air [N.m] (+200g)")
    for lab, p in CANDIDATES:
        q = D(p)
        s = TP._sigma_nd(q, P)
        sl = sig_law(q)
        t0 = g_arm(q)
        t1 = hung_tau(q)
        # standing moment change on the airframe vs home, horizontal CoM shift x weight
        dc0 = (com_offset(q) - c_home)[:2] * MTOT * G
        dc1 = (com_offset(q, MPAY) * (MTOT + MPAY) - c_home * MTOT)[:2] * G
        print(f"{lab:29s} {str(p):18s} {p[1] + p[2]:4d}  {s:.3f}  {sl:.3f}   {p[1] + 20:6.0f} {45 - p[1]:5.0f} {50 - p[2]:5.0f}   "
              f"{abs(t0[1]):.2f} {abs(t0[2]):.2f}       {abs(t1[1]):.2f} {abs(t1[2]):.2f}      "
              f"{np.linalg.norm(dc0):.2f}  ({np.linalg.norm(dc1):.2f})")

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8.5, 7))
        cs = ax.contourf(q2s, q3s, S, levels=np.linspace(0, max(0.6, S.max()), 25), cmap="viridis")
        fig.colorbar(cs, ax=ax, label="sigma_nd (planner keep-out 0.10)")
        ax.contour(q2s, q3s, S, levels=[0.10, 0.20, 0.30], colors=["red", "orange", "white"], linewidths=[2, 1.2, 1])
        ax.axvline(-20, color="k", ls="--"); ax.axvline(45, color="k", ls="--")
        ax.axhline(50, color="k", ls="--"); ax.axhline(0, color="k", ls=":")
        ax.text(-19, 51, "q3 stop +50", fontsize=8)
        ax.text(-19, 1, "q3 = 0 (the planner / teleop wall q3 >= 0)", fontsize=8)
        ax.text(-19, -13, "sigma 0.10", color="red", fontsize=8)
        ax.text(-19, 7.5, "sigma 0.20", color="orange", fontsize=8)
        ax.text(-19, 31.5, "sigma 0.30", color="white", fontsize=8)
        ax.text(-8, -36, "ELBOW SINGULAR VALLEY q3 ~ -34 (link 2 in line with wrist + gripper)",
                color="white", fontsize=8)
        mk = {"home (takeoff/landing)": "s", "pick / place": "o", "carry (place start)": "^"}
        for lab, p in CANDIDATES:
            ax.plot(p[1], p[2], mk.get(lab, "x"), color="white" if lab in mk else "#ffcccc", ms=9 if lab in mk else 6,
                    mec="k")
        ax.add_patch(plt.Rectangle((20, 33), 20, 12, fill=False, ec="white", lw=1.5, ls="-"))
        ax.text(20.3, 45.6, "task envelope flown (incl. lift / set-down)", color="white", fontsize=8)
        ax.set_xlabel("q2 [deg] (real range -20 .. +45)")
        ax.set_ylabel("q3 [deg]")
        ax.set_title("sigma_nd over q2 x q3 (q1, q4 do not change it)")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"\nwrote {a.plot}")


if __name__ == "__main__":
    main()
