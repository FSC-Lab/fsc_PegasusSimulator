#!/usr/bin/env python3
"""The design page's Isaac table (2026-11-04): this campaign's runs + the U push's matching earlier runs
(../../valve_fin_guard_20261103/sim/runs), scored the same way (sim_table.row), -> ../figures/sim_data.json.
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 sim_page.py"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_table as ST  # noqa: E402

MINE = os.path.join(HERE, "..", "sim", "runs")
UPUSH = os.path.join(HERE, "..", "..", "valve_fin_guard_20261103", "sim", "runs")
# (tag, where, the claw, the condition)
RUNS = [
    ("p1_nominal", MINE, "plate, [0, 0, 30, 0]", "nominal"),
    ("p3_drift2", MINE, "plate, [0, 0, 30, 0]", "drift seed 2"),
    ("p4_drift3", MINE, "plate, [0, 0, 30, 0]", "drift seed 3"),
    ("p5_drift4", MINE, "plate, [0, 0, 30, 0]", "drift seed 4"),
    ("p8_lat12", MINE, "plate, [0, 0, 30, 0]", "aimed 12 mm off the fin's radius"),
    ("q1_edge30", MINE, "plate, [0, 0, 30, 0]", "aimed 30 mm toward the plate's edge"),
    ("q2_high20", MINE, "plate, [0, 0, 30, 0]", "aimed 20 mm higher"),
    ("q3_noslack", MINE, "plate, [0, 0, 30, 0]", "no slack: the nose on the face at the end of Ready"),
    ("p10_pose5_25", MINE, "plate, [0, 5, 25, 0]", "nominal"),
    ("p11_pose5_25_drift2", MINE, "plate, [0, 5, 25, 0]", "drift seed 2"),
    ("p12_pose5_25_drift3", MINE, "plate, [0, 5, 25, 0]", "drift seed 3"),
    ("p13_pose5_25_drift4", MINE, "plate, [0, 5, 25, 0]", "drift seed 4"),
    ("p14_above_nominal", MINE, "plate, [0, 5, 25, 0], from above", "nominal"),
    ("p15_above_drift3", MINE, "plate, [0, 5, 25, 0], from above", "drift seed 3"),
    ("p16_above_drift4", MINE, "plate, [0, 5, 25, 0], from above", "drift seed 4"),
    ("q6_pose_m5_35", MINE, "plate, [0, −5, 35, 0]", "nominal"),
    ("p2_nominal_com", MINE, "plate, [0, 0, 30, 0], claw follows the vehicle", "nominal"),
    ("p6_drift2_com", MINE, "plate, [0, 0, 30, 0], claw follows the vehicle", "drift seed 2"),
    ("p7_drift3_com", MINE, "plate, [0, 0, 30, 0], claw follows the vehicle", "drift seed 3"),
    ("q5_drift4_com", MINE, "plate, [0, 0, 30, 0], claw follows the vehicle", "drift seed 4"),
    ("j1_nominal", UPUSH, "U push (11-03 config), [0, 5, 25, 0]", "nominal"),
    ("j4_drift2", UPUSH, "U push (11-03 config), [0, 5, 25, 0]", "drift seed 2"),
    ("g10_drift3", UPUSH, "U push (11-03 config), [0, 5, 25, 0]", "drift seed 3"),
    ("j3_drift4", UPUSH, "U push (11-03 config), [0, 5, 25, 0]", "drift seed 4"),
    ("p9_upush_lat12", MINE, "U push (11-03 config), [0, 5, 25, 0]", "aimed 12 mm off the fin's radius"),
    ("q4_upush_noslack_lat12", MINE, "U push (11-03 config), [0, 5, 25, 0]", "no slack + 12 mm off (the 11-03 situation)"),
]


def fmt_outcome(r, plate):
    if r["outcome"].startswith("completed"):
        return "completed"
    why = r.get("reason", "")
    if why.startswith("operator"):
        return "aborted by the station: the hand-over never settled within 6 s"
    why = why.replace("FIN OUT OF THE U: ", "").split(":")[0]
    import re as _re
    m = _re.match(r"q3 ([\d.]+) deg > ([\d.]+) deg", why)
    if m:
        why = f"q3 {float(m.group(1)):.1f}° over the {float(m.group(2)):.0f}° jam limit"
    if plate:
        why = why.replace("the fin top", "the plate top").replace("the fin ", "the plate ").replace("the bracket", "the nose")
    return "aborted (guard): " + why[:80]


def main():
    rows, cols = [], ["Run", "Setup", "Outcome", "Valve turned", "Peak tilt", "q3: lowest / highest in the turn",
                      "Yaw command p95 / max", "Sideways force p95", "Nose on the plate: out / up [mm]",
                      "Gear and arm to the nearest marker ball", "Before the turn: closest gap / valve moved"]
    for tag, where, claw, cond in RUNS:
        f = os.path.join(where, tag + ".npz")
        if not os.path.exists(f):
            continue
        r = ST.row(f)
        nx, nz = r.get("nose_x"), r.get("nose_z")
        nose = (f"{nx[0]:.0f}–{nx[1]:.0f} / {nz[0]:.0f}–{nz[1]:.0f}" if nx and nz else "—")
        ok = r["outcome"].startswith("completed")
        setup = claw[0].upper() + claw[1:] + " · " + cond
        q3 = f"{r.get('q3min', float('nan')):.1f}° / " + (f"{r['q3max']:.1f}°" if r.get('q3max') is not None else "—")
        rows.append({"cls": "" if ok else "warn", "cells": [
            tag, setup, fmt_outcome(r, claw.startswith('plate')), f"{r.get('valve', float('nan')):.1f}°",
            f"{r.get('tilt', float('nan')):.1f}°", q3,
            f"{r.get('tz_p95', float('nan')):.2f} / {r.get('tz_max', float('nan')):.2f} N·m",
            (f"{r['fs_p95']:.2f} N" if r.get('fs_p95') is not None else "—"), nose,
            (f"{r['clear_ball']:.0f} mm" if r.get('clear_ball') is not None else "—"),
            (f"{r['pre_gap_min']:.0f} mm" if r.get('pre_gap_min') is not None else "—") + " / " +
            (f"{r['pre_valve']:.1f}°" if r.get('pre_valve') is not None else "—")]})
        print(" | ".join(rows[-1]["cells"]))
    text = open(os.path.join(HERE, "sim_text.html")).read() if os.path.exists(os.path.join(HERE, "sim_text.html")) else ""
    note = open(os.path.join(HERE, "sim_note.html")).read() if os.path.exists(os.path.join(HERE, "sim_note.html")) else ""
    json.dump({"cols": cols, "rows": rows, "text": text, "note": note}, open(os.path.join(HERE, "..", "figures", "sim_data.json"), "w"))
    if "--md" in sys.argv:          # the same table for the README
        print("\n| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
        for r in rows:
            print("| " + " | ".join(c.replace("|", "/") for c in r["cells"]) + " |")


if __name__ == "__main__":
    main()
