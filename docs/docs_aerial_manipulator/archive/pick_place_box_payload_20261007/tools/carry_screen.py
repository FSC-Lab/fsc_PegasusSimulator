#!/usr/bin/env python3
"""Carry (place start) pose candidates for the hook-held box payload (2026-10-07):
how far the claw and the HUNG basket move from the pick / place pose, with the
constraints the hook and the real arm impose -- claw tilt (finger-top slope),
q2 / q3 stop margins, sigma_nd, and the hung basket's clearance to the skids.

    PYTHONNOUSERSITE=1 /usr/bin/python3 carry_screen.py
"""
import numpy as np
from pose_sweep import row, chain, CT, P

PICK = [0, 32, 38, 0]
HANG = 0.0176 + 0.1992 - 0.0034     # claw -> arch contact (+17.6 mm) ... arch -> basket CoM (199.2 - 3.4 mm)
SKID_X, SKID_Y, SKID_Z = 0.163, 0.142, -0.275     # skid front end, inner half-width, skid TOP (body frame, x fwd)


def claw_body(qd):
    """claw point in the ACTUAL body frame (x = nose): the model is y-forward"""
    _, r0e = CT._arm_kin(np.radians(qd), P)
    return np.array([r0e[1], -r0e[0], r0e[2]])


def basket_gap(qd):
    """hung basket (centre under the arch contact, 110 x 115 x 65) vs the skids: the
    smallest gap [m] (negative = overlapping) between the basket box and each skid box"""
    c = claw_body(qd)
    yaw = np.radians(qd[0])
    ctr = c + np.array([0.0, 0.0, 0.0176]) - np.array([0.0, 0.0, HANG])
    # basket footprint turned with the arm yaw (its x = the claw direction)
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    pts = np.array([[sx * 0.055, sy * 0.0576] for sx in (-1, 1) for sy in (-1, 1)]) @ R.T + ctr[:2]
    zlo, zhi = ctr[2] - 0.0325, ctr[2] + 0.0325
    gaps = []
    for s in (1, -1):
        lo = np.array([-SKID_X, min(s * 0.142, s * 0.168), -0.3125])
        hi = np.array([SKID_X, max(s * 0.142, s * 0.168), SKID_Z])
        blo = np.array([pts[:, 0].min(), pts[:, 1].min(), zlo]); bhi = np.array([pts[:, 0].max(), pts[:, 1].max(), zhi])
        d = np.maximum(0.0, np.maximum(blo - hi, lo - bhi))
        gaps.append(np.linalg.norm(d))
    return min(gaps), ctr


p0 = claw_body(PICK)
_, b0 = basket_gap(PICK)
print(f"pick/place {PICK}: claw {np.round(p0 * 1e3)} mm (body frame), basket centre {np.round(b0 * 1e3)} mm")
print("candidate           beta tilt  claw (x,y,z) mm        claw moves  basket moves  skid gap  sig_kin  q2 up/dn margin  q3 margin")
rows = []
for q1 in (0, 10, 15, 20, 25):
    for q2 in range(15, 41, 5):
        for q3 in range(20, 43, 2):
            q = [q1, q2, q3, 0]
            beta = q2 + q3
            if beta < 65:
                continue
            r = row(q)
            c = claw_body(q)
            g, b = basket_gap(q)
            rows.append((np.linalg.norm(b - b0), q, beta, c, np.linalg.norm(c - p0), g, r["sk"]))
rows.sort(key=lambda x: -x[0])
for bm, q, beta, c, cm, g, sk in rows[:25]:
    print(f"{str(q):19s} {beta:4d} {90 - beta:4d}  {np.round(c * 1e3).astype(int)!s:22s} {cm * 1e3:6.0f}      {bm * 1e3:6.0f}      {g * 1e3:5.0f}    {sk:.3f}    {45 - q[1]:4d} / {q[1] + 20:3d}      {50 - q[2]:3d}")
