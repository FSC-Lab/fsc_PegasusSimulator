#!/usr/bin/env python3
"""Build the push-and-pull simulation summary page (artifact) from runs/report_data.json.
    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py > ../runs/report_data.json
    /usr/bin/python3 build_summary_page.py            -> ../summary.html
"""
import base64
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CAMP = os.path.join(HERE, "..")
D = json.load(open(os.path.join(CAMP, "runs", "report_data.json")))
RUNS = {r["tag"]: r for r in D["runs"]}
L = D["L"]

# chart rows: (label, feedback, [tags whose SLIDE window is clean]); hollow = failed after the push
ROWS = [
    ("Post handle · raw · old contact timing", "raw", ["pl_38", "pl_39"]),
    ("Post handle · raw · gripper contact switch", "raw", ["pl_42", "pl_49"]),
    ("Post handle · raw · 70° fold + gripper switch", "raw", ["pl_50", "pl_51"]),
    ("Raw mocap · shipped", "raw", ["pl_15", "pl_23"]),
    ("Raw · stiffer heading (+rot. obs. 0.4)", "raw", ["pl_21", "pl_22"]),
    ("Raw · attitude from mocap", "raw", ["pl_27"]),
    ("Raw · attitude correction 1 rad/s", "raw", ["pl_29", "pl_30"]),
    ("Raw · attitude correction 3 rad/s", "raw", ["pl_32", "pl_33"]),
    ("EKF2-fused · shipped", "fused", ["pl_34"]),
    ("EKF2-fused · position trust 1 cm", "fused", ["pl_35"]),
    ("EKF2-fused · full mocap trust", "fused", ["pl_36"]),
]
chart = []
for label, fb, tags in ROWS:
    pts = []
    for t in tags:
        r = RUNS.get(t)
        if r is None or "SLIDE" not in r or (r["aborted"] and r.get("failed_in") != "release/exit"):
            continue
        pts.append(dict(tag=t, e=round(r["SLIDE"]["e_xyz_rms"], 3), rho=round(r["eps"]["SLIDE"]["rho_max"], 3),
                        F=round(r["SLIDE"]["F_rms"], 2), ey=round(r["SLIDE"]["ey_rms_mm"], 1),
                        tilt=round(r.get("push_tilt_deg", float("nan")), 1),
                        failed_after=bool(r["aborted"])))
    chart.append(dict(label=label, fb=fb, pts=pts))


def f(x, n=2):
    return "–" if x is None else f"{x:.{n}f}"


def outcome(r):
    if not r["aborted"]:
        return '<span class="chip ok">complete</span>'
    w = r.get("failed_in", "")
    return f'<span class="chip bad">failed · {w}</span>'


def vals(tags, key, sub=None, n=2, scale=1.0):
    out = []
    for t in tags:
        r = RUNS[t]
        if r["aborted"] and r.get("failed_in") not in ("release/exit",):
            continue
        v = r.get(key)
        if sub:
            v = v.get(sub) if isinstance(v, dict) else None
            if isinstance(v, dict):
                v = None
        if isinstance(v, (int, float)):
            out.append(f"{v * scale:.{n}f}")
    return ", ".join(out) if out else "–"


# configuration table: label, feedback, run tags (all), note
CONF = [
    ("Post handle + folded pose, old contact timing", "raw", ["pl_37", "pl_38", "pl_39"],
     "pl_37: plan refused at the 1.0 m takeoff (gear vs table); 1.3 m from pl_38 on"),
    ("Post handle + folded pose, gripper contact switch", "raw", ["pl_42", "pl_49"],
     "<code>push_pull_contact_at_ready</code> false, <code>_off_at_push_end</code> true"),
    ("Post handle, 70° fold + gripper switch", "raw", ["pl_50", "pl_51"],
     "pose [0, 30, 40, 0], post top 0.298 m"),
    ("Post handle + folded pose", "fused", ["pl_40", "pl_41", "pl_45", "pl_46"],
     "pl_40/41: the gear clipped the box on the way in (approach at grasp height)"),
    ("Post handle + folded pose, gripper contact switch", "fused", ["pl_47", "pl_48"], ""),
    ("Shipped push-and-pull yaml (side fin)", "raw", ["pl_9", "pl_15", "pl_23"], ""),
    ("Gain / timing changes (6 variants, one run each)", "raw",
     ["pl_20", "pl_21", "pl_22", "pl_24", "pl_25", "pl_26"],
     "rot. observer 0.4; heading K/D 0.6/0.5 (two); base damping k<sub>v</sub> 20; EE damping D<sub>y</sub> 40; push 20 s"),
    ("Attitude taken from the mocap", "raw", ["pl_27", "pl_28"], "<code>wb_attitude_from_odometry</code>"),
    ("PX4 attitude, slow error removed at 1 rad/s", "raw", ["pl_29", "pl_30", "pl_31"],
     "<code>wb_attitude_odometry_correction_rad_s</code> 1.0"),
    ("PX4 attitude, slow error removed at 3 rad/s", "raw", ["pl_32", "pl_33"], "same option, 3.0"),
    ("Shipped push-and-pull yaml", "fused", ["pl_10", "pl_19", "pl_34"], "plus the user's windowed flight (failed)"),
    ("Gain changes (7 runs)", "fused", ["pl_11", "pl_12", "pl_13", "pl_14", "pl_16", "pl_17", "pl_18"],
     "observers, contact reading 0.5 / 5.0, EE K<sub>y</sub>/D<sub>y</sub> 80/24, attitude gains"),
    ("EKF2 trusts the mocap position (1 cm)", "fused", ["pl_35"], "<code>EKF2_EVP_NOISE</code> 0.01"),
    ("EKF2 trusts mocap position, velocity, yaw", "fused", ["pl_36"],
     "<code>EKF2_EV_NOISE_MD</code> 1, EVP 0.01, EVV 0.03, EVA 0.05"),
]
conf_rows = []
for label, fb, tags, note in CONF:
    tags = [t for t in tags if t in RUNS]
    if not tags:
        continue
    n_ok = sum(1 for t in tags if not RUNS[t]["aborted"])
    n_push = sum(1 for t in tags if (not RUNS[t]["aborted"]) or RUNS[t].get("failed_in") == "release/exit")
    tag_s = ", ".join(t.replace("pl_", "") for t in tags)
    sl = vals(tags, "SLIDE", "e_xyz_rms")
    rho = []
    for t in tags:
        r = RUNS[t]
        if r["aborted"] and r.get("failed_in") != "release/exit":
            continue
        e = r.get("eps", {}).get("SLIDE")
        if e:
            rho.append(f"{e['rho_max']:.2f}")
    hold = vals(tags, "HOLD", "e_xyz_rms")
    tilt = []
    for t in tags:
        r = RUNS[t]
        if (not r["aborted"]) or r.get("failed_in") == "release/exit":
            if "push_tilt_deg" in r:
                tilt.append(f"{r['push_tilt_deg']:.0f}")
    fbc = "raw" if fb == "raw" else "fused"
    conf_rows.append(
        f'<tr><td><span class="fb {fbc}">{"raw" if fb == "raw" else "fused"}</span></td>'
        f'<td class="cfg">{label}<div class="note">{note}</div></td>'
        f'<td class="n">{n_ok} / {len(tags)}<div class="note">push done {n_push}</div></td>'
        f'<td class="n">{sl}</td><td class="n">{hold}</td><td class="n">{", ".join(rho) if rho else "–"}</td>'
        f'<td class="n">{", ".join(tilt) if tilt else "–"}</td><td class="runs">{tag_s}</td></tr>')

# per-run table
run_rows = []
order = ["pl_52", "pl_56", "pl_57", "pl_54", "pl_55", "pl_58", "pl_73", "pl_66", "pl_76", "pl_62", "pl_63", "pl_64",
         "pl_65", "pl_67", "pl_71", "pl_74", "pl_72", "pl_75", "pl_37", "pl_38", "pl_39", "pl_42", "pl_49", "pl_50", "pl_51", "pl_40", "pl_41", "pl_45", "pl_46",
         "pl_47", "pl_48", "pl_9", "pl_15", "pl_23", "pl_20", "pl_21", "pl_22", "pl_24", "pl_25", "pl_26", "pl_27", "pl_28",
         "pl_29", "pl_30", "pl_31", "pl_32", "pl_33", "pl_10", "pl_19", "pl_34", "pl_11", "pl_12", "pl_13",
         "pl_14", "pl_16", "pl_17", "pl_18", "pl_35", "pl_36"]
for t in order:
    if t not in RUNS:
        continue
    r = RUNS[t]
    clean = (not r["aborted"]) or r.get("failed_in") == "release/exit"
    s = r.get("SLIDE") if clean else None
    h = r.get("HOLD") if clean else None
    e = r.get("eps", {}).get("SLIDE") if clean else None
    run_rows.append(
        f'<tr><td class="runs">{t}</td><td><span class="fb {r["feedback"]}">{r["feedback"]}</span></td>'
        f'<td class="chg">{r["label"]}</td><td>{outcome(r)}</td>'
        f'<td class="n">{f(r.get("box_mm"), 0) if "box_mm" in r else "–"}</td>'
        f'<td class="n">{f(h["e_xyz_rms"]) if h else "–"}</td>'
        f'<td class="n">{f(s["e_xyz_rms"]) if s else "–"}</td><td class="n">{f(s["e_xyz_pk"], 1) if s else "–"}</td>'
        f'<td class="n">{f(s["e_psi_rms"], 3) if s else "–"}</td>'
        f'<td class="n">{f(s["F_rms"]) if s else "–"}</td><td class="n">{f(s["ey_rms_mm"], 1) if s else "–"}</td>'
        f'<td class="n">{f(e["max_mm"], 0) if e else "–"}</td><td class="n">{f(e["rho_max"], 3) if e else "–"}</td>'
        f'<td class="n">{f(e["rms_mm"], 0) if e else "–"}</td>'
        f'<td class="n">{f(r.get("force_angle_deg"), 0) if clean and "force_angle_deg" in r else "–"}</td>'
        f'<td class="n">{f(abs(r["pitch_moment_Nm"]), 2) if clean and "pitch_moment_Nm" in r else "–"}</td></tr>')

# design comparison (model numbers from the controller's own model; measured from the runs)
DESIGNS = [
    ("Side fin, 200 mm box", "[0, −20, 30, 0]", 0.284, 0.32, 10, ["pl_9", "pl_15", "pl_23"], ["pl_10", "pl_19", "pl_34"],
     "0.095 m", "0.29"),
    ("Post, 60° fold", "[0, 20, 40, 0]", 0.127, 0.14, 60, ["pl_38", "pl_39", "pl_42", "pl_49"],
     ["pl_40", "pl_41", "pl_45", "pl_46", "pl_47", "pl_48"], "0.230 m", "0.56"),
    ("Post, 70° fold (default)", "[0, 30, 40, 0]", 0.087, 0.10, 70, ["pl_50", "pl_51"], [], "0.273 m", "0.66"),
]
design_geom = [f'<tr><td class="cfg">{d[0]}<div class="note">pose {d[1]}°</div></td><td class="n">{d[2]:.3f} m</td>'
               f'<td class="n">{d[3]:.2f}</td><td class="n">{d[4]}°</td><td class="n">{d[7]}</td><td class="n">{d[8]}</td></tr>'
               for d in DESIGNS]


def lst(tags, fn, n=2):
    out = []
    for t in tags:
        r = RUNS.get(t)
        if r is None:
            continue
        v = fn(r)
        if v is not None:
            out.append(f"{v:.{n}f}")
    return ", ".join(out) if out else "–"


design_rows = []
for name, pose, below, mh, tiltc, raw, fused, _hg, _tip in DESIGNS:
    raw = [t for t in raw if t in RUNS]
    fused = [t for t in fused if t in RUNS]
    ok_r = sum(1 for t in raw if not RUNS[t]["aborted"])
    ok_f = sum(1 for t in fused if not RUNS[t]["aborted"])
    sl = lst(raw, lambda r: r.get("SLIDE", {}).get("e_xyz_rms") if not r["aborted"] else None)
    rho = lst(raw, lambda r: r.get("eps", {}).get("SLIDE", {}).get("rho_max") if not r["aborted"] else None)
    ang = lst(raw, lambda r: r.get("force_angle_deg") if not r["aborted"] else None, 0)
    pm = lst(raw, lambda r: abs(r["pitch_moment_Nm"]) if (not r["aborted"] and "pitch_moment_Nm" in r) else None)
    yaw = lst(raw, lambda r: r.get("box_yaw_deg") if not r["aborted"] else None, 1)
    tl = lst(raw, lambda r: r.get("push_tilt_deg") if not r["aborted"] else None, 1)
    fused_s = f"{ok_f} / {len(fused)}" if fused else "not flown"
    design_rows.append(
        f'<tr><td class="cfg">{name}</td>'
        f'<td class="n">{ok_r} / {len(raw)}</td><td class="n">{fused_s}</td>'
        f'<td class="n">{sl}</td><td class="n">{rho}</td><td class="n">{pm}</td><td class="n">{ang}</td>'
        f'<td class="n">{yaw}</td><td class="n">{tl}</td></tr>')

# ---- 2026-10-06: the real arm's q2 range [-20, 45] deg, the T650 gear, pulling ----------------
# joint window per pose, from the planner's model (transition_planner, base_com as flown): how far the
# base may sit off its plan ALONG THE ARM before a joint meets a limit, with the claw world-held
Q2_GEOM = [
    ("[0, 40, 40, 0]", 80, 0.046, "1.3", "1.6", "q2 rests 5 deg under 45"),
    ("[0, 30, 40, 0]", 70, 0.087, "3.6", "1.6", "earlier default; q3 rests 10 deg under its stop"),
    ("[0, 26, 24, 0]", 50, 0.150, "3.3", "3.5", "most balanced window"),
    ("[0, 22, 18, 0]", 40, 0.184, "2.3", "4.2", "q3 near 0: the elbow-singular branch"),
]
q2_geom = [f'<tr><td class="n">{p_}</td><td class="n">{b}°</td><td class="n">{c:.3f} m</td>'
           f'<td class="n">{a} cm</td><td class="n">{d} cm</td><td>{note}</td></tr>' for p_, b, c, a, d, note in Q2_GEOM]

# the runs: (tag, task, pose, q2 stops, note)
Q2_RUNS = [
    ("pl_52", "push", "70°", "asset (≤ 50)", ""), ("pl_56", "push", "70°", "asset (≤ 50)", ""),
    ("pl_57", "push", "70°", "asset (≤ 50)", "grasp 15 mm deeper"),
    ("pl_54", "push", "80°", "asset (≤ 50)", ""), ("pl_55", "push", "80°", "asset (≤ 50)", ""),
    ("pl_58", "push", "80°", "asset (≤ 50)", "grasp 15 mm deeper"),
    ("pl_73", "PULL", "70°", "asset (≤ 50)", ""),
    ("pl_66", "push", "70°", "−20…45", ""),
    ("pl_76", "PULL", "70°", "−20…45", ""),
    ("pl_62", "push", "50°", "−20…45", ""), ("pl_63", "push", "40°", "−20…45", ""),
    ("pl_64", "PULL", "50°", "−20…45", ""), ("pl_65", "PULL", "40°", "−20…45", ""),
    ("pl_67", "push", "50°", "−20…45", "translational observer held in contact"),
    ("pl_71", "push", "50°", "−20…45", "translational observer slowed, ω<sub>c,t</sub> 2.93 → 1.0"),
    ("pl_74", "push", "50°", "−20…45", "repeat of pl_71"),
    ("pl_72", "PULL", "50°", "−20…45", "translational observer slowed, ω<sub>c,t</sub> 2.93 → 1.0"),
    ("pl_75", "PULL", "50°", "−20…45", "repeat of pl_72"),
]


def rng(v, n=0):
    return "–" if not v else f"{v[0]:.{n}f} … {v[1]:.{n}f}"


q2_rows, q2_rows_b = [], []
for tag, task, pose, stops, note in Q2_RUNS:
    r = RUNS.get(tag)
    if r is None:
        continue
    clean = (not r["aborted"]) or r.get("failed_in") == "release/exit"
    s_ = r.get("SLIDE") if clean else None
    e_ = r.get("eps", {}).get("SLIDE") if clean else None
    out_ = outcome(r)
    if r["aborted"] and "failed_after_s" in r and r.get("failed_in", "").startswith("push"):
        out_ = f'<span class="chip bad">failed {r["failed_after_s"]:.1f} s into the push</span>'
    q2 = r.get("push_q2_deg"); q3 = r.get("push_q3_deg")
    q2s = rng(q2) + (' <b class="over">!</b>' if q2 and q2[1] > 45.05 else "")
    q3s = rng(q3) + (' <b class="over">!</b>' if q3 and q3[0] < 0 else "")
    bs = r.get("push_base_std_mm")
    box = (f(r.get("box_mm_at_fail"), 0) + " at failure" if "box_mm_at_fail" in r
           else (f(r.get("box_mm"), 0) if "box_mm" in r else "–"))
    q2_rows.append(
        f'<tr><td class="runs">{tag}</td><td>{task}</td><td class="n">{pose}</td><td class="n">{stops}</td>'
        f'<td>{out_}<div class="note">{note}</div></td><td class="n">{box}</td></tr>')
    q2_rows_b.append(
        f'<tr><td class="runs">{tag}</td><td>{task} {pose}</td>'
        f'<td class="n">{q2s}</td><td class="n">{q3s}</td>'
        f'<td class="n">{f(r.get("push_q3_stop_pct"), 0)}</td>'
        f'<td class="n">{f(bs[1], 0) if bs else "–"} mm @ {f(r.get("push_base_f_hz"), 2)} Hz</td>'
        f'<td class="n">{f(s_["e_xyz_rms"]) if s_ else "–"}</td><td class="n">{f(e_["rho_max"], 3) if e_ else "–"}</td></tr>')
refz = [RUNS[t]["ref_z_span_mm"] for t in RUNS if "ref_z_span_mm" in RUNS[t] and int(t[3:]) >= 49]
refz_s = f"{min(refz):.2f}–{max(refz):.2f}" if refz else "–"
q2_img = base64.b64encode(open(os.path.join(CAMP, "runs", "q2limit_joints.png"), "rb").read()).decode()

img = base64.b64encode(open(os.path.join(CAMP, "runs", "metrics_baseline_vs_attitude.png"), "rb").read()).decode()

TPL = open(os.path.join(HERE, "summary_page_template.html")).read()
html = (TPL.replace("@@Q2_GEOM@@", "\n".join(q2_geom))
           .replace("@@Q2_ROWS@@", "\n".join(q2_rows))
           .replace("@@Q2_ROWS_B@@", "\n".join(q2_rows_b))
           .replace("@@Q2_IMG@@", q2_img)
           .replace("@@REFZ@@", refz_s)
           .replace("@@N_RUNS@@", str(len(RUNS)))
           .replace("@@CONF_ROWS@@", "\n".join(conf_rows))
           .replace("@@DESIGN_ROWS@@", "\n".join(design_rows))
           .replace("@@DESIGN_GEOM@@", "\n".join(design_geom))
           .replace("@@RUN_ROWS@@", "\n".join(run_rows))
           .replace("@@CHART_JSON@@", json.dumps(chart))
           .replace("@@IMG@@", img)
           .replace("@@L_MM@@", f"{L * 1e3:.0f}"))
out = os.path.join(CAMP, "summary.html")
open(out, "w").write(html)
print("wrote", out, len(html) // 1024, "kB")
