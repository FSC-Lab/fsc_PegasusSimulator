#!/usr/bin/env python3
"""Build the two-part 'Sim-to-Real Flight Performance Tuning' page (2026-09-27).

    /usr/bin/python3 build_polished.py <out.html>

Part 1  the simulated plant identified from the flights: parameter table,
        sim-vs-flight snapshot curves, sim-to-real RMSE on every state
        (analysis/states_<tag>.json from tools/state_rmse.py).
Part 2  the whole-body controller after the circle tune: every parameter
        (read from the yaml files themselves), full-state circle-tracking
        RMSE before vs after (circle_tune_20260927/analysis/circle_states.json
        from its tools/circle_states.py), and the circle / q2 sweep settings.

The earlier long-form page is tools/build_report.py -> report_detailed.html.
"""
import glob
import html
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
S2R = os.path.join(HERE, "..")
AN = os.path.join(S2R, "analysis")
CT = os.path.join(S2R, "..", "circle_tune_20260927")
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
YAML_FINAL = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml")
YAML_FLOWN = os.path.join(S2R, "logs", "yaml_backup_a2_r6.yaml")      # section 2 = the hardware law as flown 09-24
YAML_BEFORE = sorted(glob.glob(os.path.join(CT, "logs", "pre_H1b_*direct_actuation_t650.yaml")))[0]


def J(path):
    return json.load(open(path))


def ykeys(path):
    d = {}
    for line in open(path):
        m = re.match(r"^\s*([A-Za-z0-9_]+):\s*([^#\n]*)", line)
        if m:
            d.setdefault(m.group(1), m.group(2).strip().strip('"'))
    return d


def f(v, n=1):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "–"
    return f"{v:.{n}f}"


E = html.escape


def wbr(k):
    return k.replace("_", "_<wbr>")

# ═══════════════════════════════════════════════════════════════ PART 1 ═════
SIM_PARAMS = [
    ("Rotors and battery", [
        ("Thrust constant k<sub>f</sub>", "sim_plant_kf_scale", "×1.037<br><span class=sub>4.19×10⁻⁵ N/(rad/s)²</span>", "×1.000<br><span class=sub>4.04×10⁻⁵</span>",
         "Hover thrust balance k<sub>f</sub> = k<sub>f,believed</sub>·mg/u<sub>1</sub> over every planner hold of six flights: 4.03–4.31×10⁻⁵, following pack voltage (23.0–24.1 V). Each replay sets its own flight's value: 0918 ×1.066, 0924 F2 ×1.030."),
        ("Battery sag", "sim_plant_kf_sag_per_min", "−3.6 %/min<br><span class=sub>from lift-off</span>", "none",
         "Linear fit of k<sub>f</sub>(t) over each DIRECT window: −2.6 to −4.5 %/min. Replays: 0918 −3.4, 0924 F2 −4.0."),
        ("Yaw torque coefficient c", "sim_plant_km_scale", "×0.2333<br><span class=sub>= 0.70 × bench c</span>", "×1.0<br><span class=sub>= 3.0 × bench c</span>",
         "Yaw acceleration per commanded yaw torque: 0.43–0.78 of the bench c; 0.78 on 0921 #3, the flight with the strongest yaw excitation."),
        ("Rotor spin-up lag λ", "sim_plant_rotor_lambda", "10.03 s⁻¹<br><span class=sub>τ 99.7 ms</span>", "10.03 s⁻¹",
         "Kept. A delay scan of the roll/pitch response finds at most 10 ms beyond the bench lag."),
    ]),
    ("Airframe", [
        ("Mass, inertia", "sim_plant_mass_scale · _inertia_scale", "×1.00 · ×1.00", "×1.00 · ×1.00",
         "Kept. Roll/pitch response per command 1.11 / 1.03 on 0921 #3, of which 1.06 is that flight's k<sub>f</sub>; at hover a mass error cannot be told apart from k<sub>f</sub>."),
        ("Bare-airframe CoM offset", "sim_plant_com_shift_x", "−17.9 mm<br><span class=sub>body x</span>", "+0.3 mm<br><span class=sub>USD asset</span>",
         "The 2026-08-25 measurement the controller already carries. The observer's hover roll/pitch residual (≤ 0.05 N·m, about 1 mm) confirms it."),
        ("Standing force, body-fixed", "sim_plant_force_bias_x · _y", "+0.55 · −0.50 N<br><span class=sub>forward · left</span>", "none",
         "Filtered d̂<sub>t</sub> in every hover hold: +0.47 to +0.62 N and −0.42 to −0.60 N. Through the 360° circle its body-frame mean stays constant to 0.1 N."),
        ("Standing yaw torque", "sim_plant_torque_bias_z", "−0.095 N·m", "none",
         "Filtered d̂<sub>r,z</sub> −0.07 to −0.13 N·m on every flight."),
    ]),
    ("Arm", [
        ("Gearbox friction, j1 · j2 · j3 · j4", "sim_arm_friction_scale_j1…j4", "1.0 · 0.70 · 0.65 · 1.5<br><span class=sub>× calibration-report model</span>", "1.0 · 1.0 · 1.0 · 1.0",
         "Kinetic friction on the measured joint velocity, 0924 circles: j2 0.11–0.17 against the model's 0.20 N·m, j3 0.03–0.09 against 0.11. j4 from the 2026-09-13 ground test; j1 has no data. The two replays in 1.2 flew the first estimate, 0.95 / 0.75."),
        ("Reported joint velocity", "sim_arm_vel_lag_s · _quant_rad_s", "48 ms lag<br><span class=sub>0.024 rad/s steps</span>", "exact PhysX rate",
         "Cross-correlation of <code>joint_states.velocity</code> with the differentiated joint position: 44–56 ms on j2/j3 in all six flights. The step size is read off the value histogram."),
        ("Current-loop residual", "sim_arm_current_noise_a_j1…j4", "9.6 · 6.2 · 9.6 · 9.6 mA<br><span class=sub>rms, 5 Hz band</span>", "same",
         "Kept from the 2026-09-11 bench. The flights' applied-minus-intended torque scatter, 0.01–0.04 N·m rms, agrees."),
        ("Arm link mass", "sim_arm_mass_scale", "×1.00", "×1.00",
         "Kept. At every hold, applied torque minus modelled gravity torque stays inside the friction band, with no consistent sign."),
    ]),
    ("Mocap feedback", [
        ("Publish rate", "sim_feedback_mocap_rate_hz", "60 Hz", "250 Hz",
         "The 0918 raw feed. On the fused stack (0924) the same feed is EKF2's external-vision input, as on hardware."),
        ("Position noise", "sim_feedback_pos_noise_m", "0.5 mm<br><span class=sub>per sample</span>", "none",
         "Sample-to-sample scatter of the raw feed. The flight's slow 0.1 Hz wander (12–17 mm) is not modelled."),
        ("Velocity noise", "sim_feedback_vel_noise_mps", "2.2 cm/s<br><span class=sub>per sample</span>", "none",
         "0918 hover: 2.6 / 2.2 / 1.5 cm/s in x / y / z."),
    ]),
]


def sim_param_table():
    rows = []
    for grp, items in SIM_PARAMS:
        rows.append(f"<tr class=grp><th colspan=5>{grp}</th></tr>")
        for name, key, val, prior, ev in items:
            rows.append(f"<tr><th>{name}</th><td class=val>{val}</td><td class=prior>{prior}</td><td class=ev>{ev}</td><td class=key><code>{wbr(key)}</code></td></tr>")
    return ("<div class=tscroll><table class=params><thead><tr><th>Parameter</th><th>Identified value</th><th>Default Isaac plant</th>"
            "<th>Evidence from the flights</th><th>yaml key</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table></div>")


STATE_ROWS = [
    ("CoM position", "mm", 1, [("x", "pos_x"), ("y", "pos_y"), ("z", "pos_z")]),
    ("CoM velocity", "mm/s", 1, [("x", "vel_x"), ("y", "vel_y"), ("z", "vel_z")]),
    ("Attitude", "deg", 2, [("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw")]),
    ("Body rate, below 5 Hz", "deg/s", 2, [("roll", "rate_p"), ("pitch", "rate_q"), ("yaw", "rate_r")]),
    ("Arm joint angle", "deg", 2, [("q1", "q1"), ("q2", "q2"), ("q3", "q3"), ("q4", "q4")]),
    ("Arm joint rate", "deg/s", 2, [("q̇1", "qd1"), ("q̇2", "qd2"), ("q̇3", "qd3"), ("q̇4", "qd4")]),
    ("Inputs", None, None, [("collective thrust u<sub>1</sub> [N]", "u1", 2), ("rotor command 1 [0–1]", "mot1", 3), ("rotor command 2", "mot2", 3),
                            ("rotor command 3", "mot3", 3), ("rotor command 4", "mot4", 3), ("joint 2 torque [N·m]", "tau2", 3), ("joint 3 torque [N·m]", "tau3", 3)]),
]


def state_rmse_table(A, F):
    head = ("<thead><tr><th rowspan=2>State</th><th colspan=3 class=cg>0924 F2 · 0.5 m circle</th><th colspan=3 class=cg>0918 · hover + base step</th></tr>"
            "<tr><th class=num>RMSE</th><th class=num>mean offset</th><th class=num>flight span</th>"
            "<th class=num>RMSE</th><th class=num>mean offset</th><th class=num>flight span</th></tr></thead>")
    rows = []
    for grp, unit, n, items in STATE_ROWS:
        rows.append(f"<tr class=grp><th colspan=7>{grp}{f' [{unit}]' if unit else ''}</th></tr>")
        for it in items:
            lab, key = it[0], it[1]
            nn = it[2] if len(it) > 2 else n
            cells = ""
            for d in (A, F):
                r = d["rmse"][key]
                cells += (f"<td class='num strong'>{f(r['rmse'], nn)}</td><td class=num>{r['bias']:+.{nn}f}</td>"
                          f"<td class='num dim'>{f(r.get('real_p1_p99'), nn)}</td>")
            rows.append(f"<tr><th class=lab>{lab}</th>{cells}</tr>")
    return f"<div class=tscroll><table class=rmse>{head}<tbody>{''.join(rows)}</tbody></table></div>"


# ------------------------------------------------------------------ figures
def ts_fig(fid, title, ylab, st, phases, key, scale=1.0, ref=None, labels=None):
    xs = {"real": [], "sim": [], "ref": []}
    ys = {"real": [], "sim": [], "ref": []}
    seps, ticks, off = [], [], 0.0
    for i, ph in enumerate(phases):
        P = st["phases"][ph]
        t = P["t"]
        tr = P["traces"]
        for side in ("real", "sim"):
            xs[side] += [round(x + off, 2) for x in t] + [None]
            ys[side] += [None if v is None or (isinstance(v, float) and not np.isfinite(v)) else round(v * scale, 4) for v in tr[key][side]] + [None]
        if ref:
            xs["ref"] += [round(x + off, 2) for x in t] + [None]
            ys["ref"] += [None if v is None or (isinstance(v, float) and not np.isfinite(v)) else round(v * scale, 4) for v in tr[ref]["real"]] + [None]
        span = t[-1] if t else 0.0
        ticks.append([round(off + span / 2, 2), (labels or phases)[i]])
        off += span + 1.5
        if i < len(phases) - 1:
            seps.append(round(off - 0.75, 2))
    return {"id": fid, "kind": "ts", "title": title, "ylab": ylab, "x": xs, "y": ys, "ref": bool(ref), "seps": seps, "ticks": ticks}


def xy_fig(fid, title, st, phase):
    tr = st["phases"][phase]["traces"]
    cl = lambda v: [None if v_ is None or (isinstance(v_, float) and not np.isfinite(v_)) else round(v_ / 1e3, 4) for v_ in v]  # noqa: E731
    return {"id": fid, "kind": "xy", "title": title,
            "x": {"real": cl(tr["pos_x"]["real"]), "sim": cl(tr["pos_x"]["sim"]), "ref": cl(tr["ref_pos_x"]["real"])},
            "y": {"real": cl(tr["pos_y"]["real"]), "sim": cl(tr["pos_y"]["sim"]), "ref": cl(tr["ref_pos_y"]["real"])}}


def figures(A, F):
    pa = ["hold0", "leg2_move", "leg2_settle"]
    la = ["hold", "circle", "hold"]
    pf = [p for p in ["hold0", "leg2_settle", "leg3_move", "leg3_settle", "leg5_settle"] if F["phases"][p]["used"]]
    lf = {"hold0": "hold", "leg2_settle": "hold", "leg3_move": "base step", "leg3_settle": "hold", "leg5_settle": "hold"}
    lf = [lf[p] for p in pf]
    circ = [
        xy_fig("c_xy", "CoM path during the circle, top view [m]", A, "leg2_move"),
        ts_fig("c_z", "CoM height from the anchor", "mm", A, pa, "pos_z", ref="ref_pos_z", labels=la),
        ts_fig("c_yaw", "Heading", "deg", A, pa, "yaw", labels=la),
        ts_fig("c_roll", "Roll", "deg", A, pa, "roll", labels=la),
        ts_fig("c_pitch", "Pitch", "deg", A, pa, "pitch", labels=la),
        ts_fig("c_q2", "Arm joint q2", "deg", A, pa, "q2", ref="ref_q2", labels=la),
        ts_fig("c_q3", "Arm joint q3", "deg", A, pa, "q3", ref="ref_q3", labels=la),
        ts_fig("c_u1", "Collective thrust u₁", "N", A, pa, "u1", labels=la),
        ts_fig("c_dx", "Observer force estimate d̂ₓ (world)", "N", A, pa, "dhat_x", labels=la),
    ]
    step = [
        ts_fig("f_x", "CoM x from the anchor", "mm", F, pf, "pos_x", ref="ref_pos_x", labels=lf),
        ts_fig("f_z", "CoM height from the anchor", "mm", F, pf, "pos_z", ref="ref_pos_z", labels=lf),
        ts_fig("f_pitch", "Pitch", "deg", F, pf, "pitch", labels=lf),
        ts_fig("f_u1", "Collective thrust u₁", "N", F, pf, "u1", labels=lf),
        ts_fig("f_dz", "Observer force estimate d̂_z", "N", F, pf, "dhat_z", labels=lf),
        ts_fig("f_vx", "CoM velocity x", "mm/s", F, pf, "vel_x", labels=lf),
    ]
    return circ, step


# ═══════════════════════════════════════════════════════════════ PART 2 ═════
def ctrl_rows():
    fl, be, fi = ykeys(YAML_FLOWN), ykeys(YAML_BEFORE), ykeys(YAML_FINAL)

    def g(d, *ks, fmt=None, sep=" · "):
        vals = [d.get(k) for k in ks]
        if any(v is None for v in vals):
            return None
        if fmt:
            vals = [fmt(v) for v in vals]
        return sep.join(vals)

    num = lambda p: (lambda v: f"{float(v):.{p}f}")  # noqa: E731
    onoff = lambda v: "on" if v.lower() == "true" else "off"  # noqa: E731
    R = []

    def row(label, sym, keys, unit="", fmt=None, flown=None, before=None, final=None, note="", same=False):
        a = flown if flown is not None else g(fl, *keys, fmt=fmt)
        b = before if before is not None else g(be, *keys, fmt=fmt)
        c = final if final is not None else g(fi, *keys, fmt=fmt)
        R.append((label, sym, unit, a, b, c, note, keys))

    grp = lambda name: R.append(("__grp__", name))  # noqa: E731
    grp("Translational (CoM) loop")
    row("Position gain", "k<sub>x</sub>", ["wb_k_x"], "N/(m·kg)", num(2))
    row("Velocity gain", "k<sub>v</sub>", ["wb_k_v"], "N·s/(m·kg)", num(2))
    grp("Attitude loop")
    row("Attitude gain", "k<sub>R</sub>", ["wb_k_r"], "", num(3))
    row("Angular-rate gain", "k<sub>ω</sub>", ["wb_k_w"], "", num(3))
    row("Desired rotational inertia", "M<sub>r,d</sub> x · y · z", ["wb_mrd_x", "wb_mrd_y", "wb_mrd_z"], "kg·m²", num(4))
    grp("End-effector impedance")
    row("Task stiffness, position", "K<sub>y</sub> x = y = z", ["wb_ky_x"], "N/m", num(1))
    row("Task stiffness, heading", "K<sub>ψ</sub>", ["wb_ky_psi"], "N·m/rad", num(3))
    row("Task damping, position", "D<sub>y</sub> x = y = z", ["wb_dy_x"], "N·s/m", num(2))
    row("Task damping, heading", "D<sub>ψ</sub>", ["wb_dy_psi"], "N·m·s/rad", num(3))
    row("Desired task inertia", "M<sub>y</sub> x · y · z · ψ", ["wb_my_x", "wb_my_y", "wb_my_z", "wb_my_psi"], "kg, kg·m²", num(2))
    row("Inertia shaping", "", ["wb_impedance_shaped"], "", onoff)
    row("Damped least-squares factor", "λ<sub>DLS</sub>", ["wb_dls_lambda"], "", num(2))
    grp("Arm channel and model")
    row("Servo armature in the model", "J<sub>arm</sub> j1 · j2 · j3 · j4", ["wb_armature_j1", "wb_armature_j2", "wb_armature_j3", "wb_armature_j4"], "kg·m²",
        num(4), flown="0.020 each, on the child link", note="joint-diagonal since 09-26")
    row("Joint velocity fed to the law", "q̇", ["wb_arm_velocity_topic"], "", lambda v: "velocity observer, 12 ms",
        flown="servo Present Velocity, ≈48 ms")
    row("Joint torque limit", "τ<sub>max</sub>", ["wb_tau_max"], "N·m", num(1))
    row("EE reference held about the CoM", "", ["wb_ee_anchor_com"], "", onoff)
    row("Internal-disturbance feed-forward in u₃, from ŵ", "", ["wb_u3_internal_ff"], "", onoff)
    row("Estimate cap on u₃, j4", "", ["wb_u3_estimate_max_j4"], "N·m", num(2))
    row("Bare-airframe CoM, model y", "", ["wb_base_com_y"], "m", num(6))
    grp("L1 disturbance observer, 4-D attribution")
    row("Adaptation poles", "A<sub>s</sub> t · r · q", ["wb_l1_a_t", "wb_l1_a_r", "wb_l1_a_q"], "s⁻¹", num(1))
    row("Filter bandwidth, translation", "ω<sub>c,t</sub>", ["wb_l1_omega_c_t"], "rad/s", num(3))
    row("Filter bandwidth, rotation", "ω<sub>c,r</sub>", ["wb_l1_omega_c_r"], "rad/s", num(3))
    row("Filter bandwidth, arm joints", "ω<sub>c,q</sub>", ["wb_l1_omega_c_q"], "rad/s", num(3))
    row("Feed-forward filter", "ω<sub>x</sub>", ["wb_l1_omega_x"], "rad/s", num(3))
    row("Joint-trim filter", "ω<sub>q</sub>", ["wb_l1_omega_q"], "rad/s", num(2))
    row("Estimate bounds", "force · torque · joint", ["wb_l1_max_force_n", "wb_l1_max_torque_nm", "wb_l1_max_joint_nm"], "N · N·m · N·m", num(1))
    row("Contact mode", "", ["wb_l1_contact"], "", lambda v: "free flight" if v.lower() == "false" else "contact")
    grp("Allocation and vehicle model")
    row("Believed thrust constant", "k<sub>f</sub>", ["alloc_thrust_coeff"], "N/(rad/s)²", lambda v: v)
    row("Yaw torque ratio", "k<sub>m</sub>", ["alloc_rotor0_km"], "m", num(6))
    row("Rotor arm (X layout)", "", ["alloc_rotor0_px"], "m", lambda v: f"±{float(v):.4f}")
    row("Rotor speed idle · max", "ω", ["alloc_omega_idle", "alloc_omega_max"], "rad/s", num(2))
    row("Vehicle mass", "m", ["vehicle_mass"], "kg", num(5))
    grp("Guards and DIRECT entry")
    row("Tilt · rate watchdog", "", ["system_wd_max_tilt_deg", "system_wd_max_rate_dps"], "deg · deg/s", num(0))
    row("Horizontal drift trip", "", ["system_wd_max_drift_m"], "m", num(2))
    row("Feedback timeout", "", ["system_fb_timeout_s"], "s", num(1))
    row("Entry gates: position · velocity · arm", "", ["wb_gate_pos_m", "wb_gate_vel_mps", "wb_gate_arm_rad"], "m · m/s · rad", num(2))
    row("SAFETY arm hold PD", "k<sub>p</sub> · k<sub>d</sub>", ["wb_arm_hold_kp", "wb_arm_hold_kd"], "N·m/rad · N·m·s/rad", num(2))
    return R


def changed_by(a, b, c, keys):
    s = []
    if a is not None and b is not None and a != b:
        s.append("k<sub>ω</sub> study, 09-26" if keys == ["wb_k_w"] else "arm-channel fix, 09-26")
    if b is not None and c is not None and b != c:
        s.append("circle tune, 09-27")
    return " → ".join(s) if s else "—"


def ctrl_table():
    rows = []
    for r in ctrl_rows():
        if r[0] == "__grp__":
            rows.append(f"<tr class=grp><th colspan=5>{r[1]}</th></tr>")
            continue
        label, sym, unit, a, b, c, note, keys = r
        tuned = b != c
        cls = " class=tuned" if tuned else ""
        symu = " · ".join(x for x in (sym, f"[{unit}]" if unit else "") if x)
        rows.append(f"<tr{cls}><th>{label}{f'<span class=sym>{symu}</span>' if symu else ''}</th>"
                    f"<td class='num dim'>{a if a is not None else '–'}</td><td class='num dim'>{b if b is not None else '–'}</td>"
                    f"<td class='num fin'>{c if c is not None else '–'}</td><td class=chg>{changed_by(a, b, c, keys)}</td></tr>")
    return ("<div class=tscroll><table class=ctrl><thead><tr><th>Parameter</th><th class=num>Flown 09-18 … 09-24</th>"
            "<th class=num>Before the circle tune</th><th class=num>Final</th><th>Changed by</th></tr></thead><tbody>" + "".join(rows) + "</tbody></table></div>")


TRACK_ROWS = [
    ("CoM position", "mm", 1, [("x", "com_pos_x"), ("y", "com_pos_y"), ("z", "com_pos_z"), ("norm", "com_pos_norm")]),
    ("CoM velocity", "mm/s", 1, [("x", "com_vel_x"), ("y", "com_vel_y"), ("z", "com_vel_z"), ("norm", "com_vel_norm")]),
    ("Attitude", "deg", 2, [("roll", "att_roll"), ("pitch", "att_pitch"), ("yaw", "att_yaw"), ("norm", "att_norm")]),
    ("Body rate", "deg/s", 2, [("roll", "rate_roll"), ("pitch", "rate_pitch"), ("yaw", "rate_yaw"), ("norm", "rate_norm")]),
    ("End-effector position", "mm", 1, [("x", "ee_pos_x"), ("y", "ee_pos_y"), ("z", "ee_pos_z"), ("norm", "ee_pos_norm")]),
    ("End-effector heading", "deg", 2, [("ψ<sub>e</sub>", "ee_head")]),
    ("Arm joint angle", "deg", 2, [("q1", "q1"), ("q2", "q2"), ("q3", "q3"), ("q4", "q4")]),
    ("Arm joint rate", "deg/s", 2, [("q̇1", "qd1"), ("q̇2", "qd2"), ("q̇3", "qd3"), ("q̇4", "qd4")]),
]
CASES = [("flown", "Flown circle · r 0.5 m"), ("show", "Showcase circle · r 0.75 m"), ("show_rob", "Showcase · robustness plant")]


def delta_cell(a, b):
    if a is None or b is None or a == 0:
        return "<td class=d>–</td>"
    p = 100.0 * (b - a) / a
    cls = "ok" if p <= -3 else ("warn" if p >= 3 else "")
    return f"<td class='d {cls}'>{'−' if p < 0 else '+'}{abs(p):.0f} %</td>"


def track_table(src, D):
    head = ("<thead><tr><th rowspan=2>State</th>" + "".join(f"<th colspan=3 class=cg>{t}</th>" for _, t in CASES) + "</tr><tr>"
            + "<th class=num>before</th><th class=num>after</th><th class=num>change</th>" * len(CASES) + "</tr></thead>")
    rows = []
    for grp, unit, n, items in TRACK_ROWS:
        rows.append(f"<tr class=grp><th colspan=10>{grp} [{unit}]</th></tr>")
        for lab, key in items:
            cells = ""
            for c, _ in CASES:
                a = D[c]["ship"].get(key); b = D[c]["h1b"].get(key)
                if a is None:
                    cells += "<td class='num dim na' colspan=3>not logged</td>"
                    continue
                cells += f"<td class='num dim'>{f(a, n)}</td><td class='num strong'>{f(b, n)}</td>{delta_cell(a, b)}"
            cls = " class=norm" if lab == "norm" else ""
            rows.append(f"<tr{cls}><th class=lab>{'|·| norm' if lab == 'norm' else lab}</th>{cells}</tr>")
    # run facts
    rows.append("<tr class=grp><th colspan=10>Run</th></tr>")
    cells = ""
    for c, _ in CASES:
        for law in ("ship", "h1b"):
            m = D[c][law]["_meta"]
            if src == "isaac":
                txt = f"{'aborted' if m['aborted'] else 'completed'}<br>sat {m['sat_pct']:.0f} %, clamp {m['clamp_pct']:.0f} %<br>|τ| {m['tau_pk']:.2f} N·m"
            else:
                txt = f"{m['seeds']}/3 completed<br>sat {m['sat_pct']:.0f} %<br>|τ| {m['tau_pk']:.2f} N·m"
            cells += f"<td class='num run'>{txt}</td>"
        cells += "<td></td>"
    rows.append(f"<tr><th class=lab>outcome<span class=sub><br>peak arm |τ|</span></th>{cells}</tr>")
    return f"<div class=tscroll><table class=track>{head}<tbody>{''.join(rows)}</tbody></table></div>"


def traj_tables():
    circ = [
        ("Radius", "ee_traj_circle_radius", "0.50 m", "<b>0.75 m</b>"),
        ("Lap time at time scale 1", "ee_traj_lap_time", "24 s", "24 s"),
        ("Laps", "ee_traj_laps", "1", "1"),
        ("Direction (seen from above)", "ee_traj_ccw", "counter-clockwise", "<b>clockwise</b>"),
        ("Ramp-in and ramp-out", "ee_traj_ramp_time", "4 s each, min-snap", "4 s each, min-snap"),
        ("Centre", "ee_traj_center_origin", "world origin, at the EE height", "world origin, at the EE height"),
        ("Time scale", "ee_traj_time_scale", "1.0", "1.0"),
        ("EE heading", "—", "along the tangent", "along the tangent"),
        ("Mean EE speed (derived)", "", "0.131 m/s", "0.196 m/s"),
        ("Heading rate (derived)", "", "15 °/s", "15 °/s"),
    ]
    q2 = [
        ("Arm fold β = q2 + q3", "ee_traj_fold_deg", "60°", "60°"),
        ("q2 centre", "ee_traj_q2_center_deg", "30°", "30°"),
        ("q2 amplitude", "ee_traj_q2_amp_deg", "±10°", "<b>±15°</b>"),
        ("q2 period", "ee_traj_q2_period_s", "48 s (half a sine per lap)", "<b>24 s (one sine per lap)</b>"),
        ("q2 range (derived)", "", "20° … 40°", "15° … 45°"),
        ("q3 range, = β − q2 (derived)", "", "20° … 40°", "15° … 45°"),
        ("Peak q2 rate (derived)", "", "1.3 °/s", "3.9 °/s"),
        ("q1 (arm yaw)", "", "0, fixed", "0, fixed"),
        ("q4 (wrist roll)", "", "solved for the EE heading", "solved for the EE heading"),
        ("Joint-rate limit", "ee_traj_qdot_max", "0.5 rad/s", "0.5 rad/s"),
    ]

    def t(rows, cap):
        body = "".join(f"<tr><th>{a}</th><td class=key>{'<code>' + wbr(k) + '</code>' if k and k != '—' else ''}</td><td>{b}</td><td>{c}</td></tr>" for a, k, b, c in rows)
        return (f"<div class=tscroll><table class=traj><caption>{cap}</caption><thead><tr><th>Parameter</th><th>yaml key</th>"
                f"<th>Flown 09-24 (current default)</th><th>Recommended showcase</th></tr></thead><tbody>{body}</tbody></table></div>")
    return t(circ, "Circle"), t(q2, "q2 sweep (the arm's redundant degree of freedom)")


# ═══════════════════════════════════════════════════════════════ PAGE ═══════
CSS = """
:root{--bg:#F3F5F4;--panel:#FFFFFF;--ink:#16222A;--ink2:#46565E;--ink3:#77878E;--line:#D6DEE0;--line2:#E8EEEF;
--real:#1E6B76;--sim:#B4261F;--tint:#E3EEF0;--ok:#2C7A57;--warn:#A8620A;--mark:#FFF3D6;
--mono:"IBM Plex Mono",ui-monospace,Menlo,monospace;--sans:"IBM Plex Sans",system-ui,-apple-system,sans-serif;--disp:"IBM Plex Sans Condensed","IBM Plex Sans",system-ui,sans-serif}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#11181C;--panel:#182228;--ink:#E4EAEC;--ink2:#A9B6BC;--ink3:#7B8A91;--line:#2B3940;--line2:#223036;--real:#6FC3CE;--sim:#F08A83;--tint:#1B3239;--ok:#6CC499;--warn:#E3A24B;--mark:#3A3220;color-scheme:dark}}
:root[data-theme="dark"]{--bg:#11181C;--panel:#182228;--ink:#E4EAEC;--ink2:#A9B6BC;--ink3:#7B8A91;--line:#2B3940;--line2:#223036;--real:#6FC3CE;--sim:#F08A83;--tint:#1B3239;--ok:#6CC499;--warn:#E3A24B;--mark:#3A3220;color-scheme:dark}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);font-family:var(--sans);font-size:15.5px;line-height:1.55;margin:0;padding-inline:20px;padding-block:36px 72px}
.wrap{max-width:1120px;margin:0 auto;display:grid;gap:56px}
h1,h2,h3{font-family:var(--disp);font-weight:600;line-height:1.15;text-wrap:balance;margin:0}
h1{font-size:2.35rem;letter-spacing:-.005em}
h2{font-size:1.7rem}
h3{font-size:1.15rem}
p{margin:0;max-width:74ch}
.eyebrow{font-family:var(--mono);font-size:.76rem;letter-spacing:.08em;text-transform:uppercase;color:var(--ink3)}
header{display:grid;gap:14px}
.meta{display:flex;flex-wrap:wrap;gap:6px 26px;font-family:var(--mono);font-size:.8rem;color:var(--ink2)}
.meta b{color:var(--ink);font-weight:500}
.part{display:grid;gap:34px}
.parthead{display:grid;grid-template-columns:auto 1fr;gap:4px 18px;align-items:baseline;border-top:3px solid var(--ink);padding-top:14px}
.parthead .no{font-family:var(--disp);font-size:3.2rem;font-weight:600;line-height:.9;color:var(--real);grid-row:span 2}
.parthead p{color:var(--ink2);grid-column:2}
section{display:grid;gap:14px}
.sechead{display:flex;gap:12px;align-items:baseline;flex-wrap:wrap}
.sechead .sn{font-family:var(--mono);font-size:.85rem;color:var(--ink3)}
.lead{color:var(--ink2);font-size:.95rem}
ul.notes{margin:0;padding-left:1.1em;display:grid;gap:7px;max-width:84ch;font-size:.93rem;color:var(--ink2)}
ul.notes b{color:var(--ink);font-weight:500}
li::marker{color:var(--ink3)}
code{font-family:var(--mono);font-size:.84em;background:var(--tint);padding:1px 5px;border-radius:3px;color:var(--ink);overflow-wrap:anywhere}
.tscroll{overflow-x:auto;background:var(--panel);border:1px solid var(--line)}
table{border-collapse:collapse;width:100%;font-size:.86rem;font-variant-numeric:tabular-nums}
caption{text-align:left;font-family:var(--disp);font-weight:600;font-size:1rem;padding:12px 12px 4px;color:var(--ink)}
th,td{text-align:left;padding:6px 10px;border-bottom:1px solid var(--line2);vertical-align:top}
thead th{font-family:var(--mono);font-size:.7rem;letter-spacing:.05em;text-transform:uppercase;color:var(--ink3);font-weight:500;border-bottom:1px solid var(--line);white-space:nowrap}
thead th.cg{text-align:center;color:var(--ink2);border-bottom:1px solid var(--line);border-left:1px solid var(--line)}
tbody th{font-weight:500}
tr.grp th{font-family:var(--mono);font-size:.7rem;letter-spacing:.07em;text-transform:uppercase;color:var(--real);background:var(--tint);padding-top:7px;padding-bottom:5px;border-bottom:none}
td.num,th.num{text-align:right;font-family:var(--mono);white-space:nowrap}
td.dim{color:var(--ink3)} td.strong{font-weight:500}
th.lab{font-weight:400;color:var(--ink2);padding-left:18px;white-space:nowrap}
tr.norm th.lab{color:var(--ink);font-weight:500} tr.norm td{border-bottom:1px solid var(--line)}
td.d{font-family:var(--mono);font-size:.78rem;text-align:right;white-space:nowrap;color:var(--ink3)}
td.d.ok{color:var(--ok);font-weight:500} td.d.warn{color:var(--warn);font-weight:500}
table.rmse td:nth-child(2),table.rmse td:nth-child(5),table.track td:nth-child(2),table.track td:nth-child(5),table.track td:nth-child(8){border-left:1px solid var(--line)}
table.track td.run{font-size:.72rem;line-height:1.4;color:var(--ink2);white-space:nowrap}
table.params td.val{font-family:var(--mono);font-weight:500;white-space:nowrap}
table.params td.prior{font-family:var(--mono);color:var(--ink3);white-space:nowrap}
table.params td.ev{min-width:300px;color:var(--ink2);font-size:.83rem;line-height:1.45}
table.params td.key code,table.traj td.key code{font-size:.72rem;background:none;padding:0;color:var(--ink3);overflow-wrap:normal}
table.params tbody th{min-width:130px}
td.na{text-align:center;font-size:.76rem}
table.traj tbody th{min-width:190px}
.sub{font-family:var(--mono);font-size:.72rem;color:var(--ink3);font-weight:400}
table.ctrl tbody th{min-width:250px}
table.ctrl th .sym{display:block;font-family:var(--mono);font-size:.72rem;color:var(--ink3);font-weight:400;margin-top:1px}
table.ctrl td.num{white-space:normal}
table.ctrl td.fin{font-weight:500}
table.ctrl tr.tuned td.fin{background:var(--mark)}
table.ctrl td.chg{font-size:.78rem;color:var(--ink2);min-width:150px}
table.traj td{white-space:nowrap}
.tables2{display:grid;gap:18px}
.legend{display:flex;gap:20px;flex-wrap:wrap;font-family:var(--mono);font-size:.78rem;color:var(--ink2)}
.legend i{display:inline-block;width:24px;height:0;border-top:2.5px solid;vertical-align:middle;margin-right:7px}
.figgroup{display:grid;gap:10px}
.figgroup h4{margin:0;font-family:var(--disp);font-size:1rem;font-weight:600}
.figgroup h4 span{font-family:var(--sans);font-weight:400;color:var(--ink3);font-size:.88rem}
.figs{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,320px),1fr));gap:10px}
.fig{background:var(--panel);border:1px solid var(--line);padding:6px 4px 0;min-width:0}
.fig .ttl{font-family:var(--mono);font-size:.74rem;color:var(--ink2);padding:2px 10px 0}
.fig .plot{width:100%;height:210px}
.fig.xy .plot{height:260px}
.callout{border-left:3px solid var(--real);background:var(--panel);padding:10px 14px;font-size:.92rem;color:var(--ink2);max-width:84ch}
.callout b{color:var(--ink);font-weight:500}
footer{font-size:.82rem;color:var(--ink3);display:grid;gap:4px;border-top:1px solid var(--line);padding-top:14px}
footer code{font-size:.78rem}
@media (max-width:560px){h1{font-size:1.8rem}.parthead .no{font-size:2.4rem}body{padding-inline:16px}}
"""

JS = r"""
const V=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
function draw(f){
  const real=V('--real'),sim=V('--sim'),ink=V('--ink2'),ink3=V('--ink3'),line=V('--line2'),panel=V('--panel');
  const base={margin:{l:48,r:10,t:10,b:30},paper_bgcolor:panel,plot_bgcolor:panel,showlegend:false,
    font:{family:'IBM Plex Mono, monospace',size:10,color:ink}};
  let tr,lay;
  if(f.kind==='xy'){
    tr=[{x:f.x.ref,y:f.y.ref,mode:'lines',line:{color:ink3,width:1.2,dash:'dot'},hoverinfo:'skip'},
        {x:f.x.real,y:f.y.real,mode:'lines',line:{color:real,width:2},name:'flight',hovertemplate:'flight %{x:.3f}, %{y:.3f} m<extra></extra>'},
        {x:f.x.sim,y:f.y.sim,mode:'lines',line:{color:sim,width:1.7},name:'sim',hovertemplate:'sim %{x:.3f}, %{y:.3f} m<extra></extra>'}];
    lay=Object.assign({},base,{margin:{l:48,r:10,t:10,b:34},xaxis:{title:{text:'x [m]',standoff:2},gridcolor:line,zeroline:false},
        yaxis:{title:{text:'y [m]',standoff:2},gridcolor:line,zeroline:false,scaleanchor:'x',scaleratio:1},hovermode:'closest'});
  }else{
    tr=[];
    if(f.ref){tr.push({x:f.x.ref,y:f.y.ref,mode:'lines',line:{color:ink3,width:1.2,dash:'dot'},name:'plan',hovertemplate:'plan %{y:.3~f}<extra></extra>'});}
    tr.push({x:f.x.real,y:f.y.real,mode:'lines',line:{color:real,width:1.9},name:'flight',hovertemplate:'flight %{y:.3~f}<extra></extra>'});
    tr.push({x:f.x.sim,y:f.y.sim,mode:'lines',line:{color:sim,width:1.6},name:'sim',hovertemplate:'sim %{y:.3~f}<extra></extra>'});
    const shapes=f.seps.map(s=>({type:'line',x0:s,x1:s,y0:0,y1:1,yref:'paper',line:{color:V('--line'),width:1}}));
    lay=Object.assign({},base,{xaxis:{tickmode:'array',tickvals:f.ticks.map(t=>t[0]),ticktext:f.ticks.map(t=>t[1]),gridcolor:line,zeroline:false,showgrid:false},
        yaxis:{title:{text:f.ylab,standoff:2},gridcolor:line,zeroline:false},shapes:shapes,hovermode:'x unified',
        hoverlabel:{bgcolor:panel,bordercolor:V('--line'),font:{color:V('--ink')}}});
  }
  Plotly.react(f.id,tr,lay,{displaylogo:false,responsive:true,displayModeBar:false});
}
function drawAll(){if(typeof Plotly==='undefined'){document.querySelectorAll('.fig .plot').forEach(e=>{e.textContent='Plotly did not load, so the figures are unavailable.';e.style.cssText+='padding:12px;font-size:12px;color:'+V('--ink3');});return;}FIGS.forEach(draw);}
window.addEventListener('load',drawAll);
try{matchMedia('(prefers-color-scheme: dark)').addEventListener('change',drawAll);}catch(e){}
new MutationObserver(drawAll).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
"""


def fig_html(fs):
    return "".join(f"<div class='fig{' xy' if f_['kind'] == 'xy' else ''}'><div class=ttl>{f_['title']}{' [' + f_['ylab'] + ']' if f_['kind'] == 'ts' else ''}</div>"
                   f"<div class=plot id='{f_['id']}' role=img aria-label='{E(f_['title'])}: flight vs simulation'></div></div>" for f_ in fs)


def main():
    out = sys.argv[1]
    A = J(os.path.join(AN, "states_a2_r6.json"))
    F = J(os.path.join(AN, "states_f18_r4.json"))
    CA = J(os.path.join(AN, "compare_a2_r6.json"))["phases"]["leg2_move"]
    CS = J(os.path.join(CT, "analysis", "circle_states.json"))
    circ, step = figures(A, F)
    tb_circle, tb_q2 = traj_tables()
    ck = CS["isaac_clock_check"]
    I = CS["isaac"]
    B = CS["bench"]
    exA = ", ".join(f"{x['phase']}" for x in A["excluded"])
    exF = [x for x in F["excluded"] if x["reason"] == "plans differ"]
    qdiff = (min(x["ref_q_rmse_deg"] for x in exF), max(x["ref_q_rmse_deg"] for x in exF))
    pct = lambda a, b: 100 * (b - a) / a  # noqa: E731
    ra = lambda k: A["rmse"][k]["rmse"]  # noqa: E731
    rf = lambda k: F["rmse"][k]["rmse"]  # noqa: E731
    sd = lambda D, k: float(np.sqrt(max(D["rmse"][k]["rmse"] ** 2 - D["rmse"][k]["bias"] ** 2, 0.0)))  # noqa: E731

    def rng(key, cases=("flown", "show")):
        v = sorted(abs(pct(B[c]["ship"][key], B[c]["h1b"][key])) for c in cases)
        return f"{v[0]:.0f}–{v[-1]:.0f} %" if round(v[0]) != round(v[-1]) else f"{v[0]:.0f} %"

    def rng_many(keys, cases=("flown", "show")):
        v = sorted(abs(pct(B[c]["ship"][k], B[c]["h1b"][k])) for c in cases for k in keys)
        return f"{v[0]:.0f}–{v[-1]:.0f} %"

    page = f"""<title>Sim-to-Real Flight Performance Tuning</title>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:ital,wght@0,400;0,500;1,400&family=IBM+Plex+Mono:wght@400;500&display=swap">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>{CSS}</style>
<div class=wrap>
<header>
  <div class=eyebrow>AM-T650 aerial manipulator · whole-body L1-adaptive controller, 4-D attribution · Isaac Sim</div>
  <h1>Sim-to-Real Flight Performance Tuning</h1>
  <p>Part 1 identifies the simulated plant from the 0918, 0921 and 0924 indoor flights and measures how closely Isaac Sim reproduces the flown states. Part 2 lists the whole-body controller after the circle-tracking tune and compares tracking error on the end-effector circle before and after it, state by state.</p>
  <div class=meta><span>flights <b>0918 · 0921 #3 · 0924 F1–F4</b></span><span>simulator <b>Isaac Sim + PX4 SITL, RTF 0.48</b></span><span>tuned gains <b>in the hardware yaml, not yet flown</b></span><span>updated <b>2026-09-27</b></span></div>
</header>

<div class=part>
  <div class=parthead><div class=no>1</div><h2>Simulation plant identified from the flights</h2>
    <p>Only the plant moved. The simulator's controller section is the hardware file verbatim, so every difference below is the plant or the feedback.</p></div>

  <section>
    <div class=sechead><span class=sn>1.1</span><h3>Simulation parameters tuned from flight data</h3></div>
    <p class=lead>Identified values live in section 1 of <code>params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml</code>. Control parameters are excluded here; they are in 2.1.</p>
    {sim_param_table()}
    <div class=callout><b>Isaac's clock, not a plant parameter.</b> Isaac and PX4 run at a real-time factor of about 0.48 on this machine while the controller and planner run on the wall clock. The launcher therefore divides the rotor lag rate λ by the RTF, multiplies the joint-velocity lag by it, and rescales the noise band (<code>sim_wall_clock_compensation: true</code>, <code>SIM_RTF 0.48</code>); the planner's time scale is set to the RTF. These settings keep plant time consistent. They were not identified from the flights.</div>
  </section>

  <section>
    <div class=sechead><span class=sn>1.2</span><h3>Simulation against flight</h3></div>
    <p class=lead>Each flight was replayed in Isaac with its own reference, the as-flown controller (k<sub>ω</sub> 1.5, K<sub>y</sub> 20) and the identified plant. The sim's time is plant time. Each planner leg is warped onto the flight's leg by the plan's own path progress. The sim's circle, anchored at a different place and heading, is rotated {abs(A['rigid']['yaw_deg']):.1f}° and shifted onto the flight's; after that the two plans agree to {A['phases']['leg2_move']['ref_pos_rmse_mm']:.1f} mm.</p>
    <div class=legend><span><i style="border-color:var(--real)"></i>flight</span><span><i style="border-color:var(--sim)"></i>simulation</span><span><i style="border-color:var(--ink3);border-top-style:dotted"></i>plan reference</span></div>
    <div class=figgroup><h4>0924 F2 · 0.5 m end-effector circle <span>EKF2-fused feedback, entry hold → circle → hold</span></h4><div class=figs>{fig_html(circ)}</div></div>
    <div class=figgroup><h4>0918 · hover and 0.61 m base step <span>raw 60 Hz mocap feedback; only legs where both runs flew the same plan</span></h4><div class=figs>{fig_html(step)}</div></div>
  </section>

  <section>
    <div class=sechead><span class=sn>1.3</span><h3>Sim-to-real RMSE, full state</h3></div>
    <p class=lead>RMSE of the simulated state minus the flown state over the compared phases, on the same 50 Hz grid. The mean offset is the constant part of that difference. The flight span (1st–99th percentile of the flown state) gives the scale.</p>
    {state_rmse_table(A, F)}
    <ul class=notes>
      <li><b>Compared:</b> {A['compared_s']:.1f} s of the circle mission (entry hold, the circle, the closing hold) and {F['compared_s']:.1f} s of the 0918 mission (the entry hold, the base step and the holds around it). Left out: the circle's go-to-start turn, because the two runs began at different headings ({A['excluded'][0]['ref_pos_rmse_mm']:.0f} mm plan difference); 0918's three arm legs, because the replay's planner resolved the same end-effector targets to joint poses {qdiff[0]:.1f}–{qdiff[1]:.1f}° apart; and the 0918 hold that contains the flight's 2.2 s mocap dropout (44.6–46.9 s), a feedback fault rather than plant behaviour.</li>
      <li><b>Circle position, {min(ra('pos_x'), ra('pos_y')):.0f}–{max(ra('pos_x'), ra('pos_y')):.0f} mm:</b> the simulation tracks the circle less tightly than the flight (CoM error {CA['sim']['com_err_norm_rms_mm']:.0f} against {CA['real']['com_err_norm_rms_mm']:.0f} mm rms). Most of the excess is Isaac's wall-clock coupling, measured in 2.3. Height agrees within {ra('pos_z'):.0f} mm, thrust within {ra('u1'):.1f} N and the rotor commands within {max(ra('mot1'), ra('mot2'), ra('mot3'), ra('mot4')):.3f}.</li>
      <li><b>0918 position, {min(rf('pos_x'), rf('pos_y')):.0f}–{max(rf('pos_x'), rf('pos_y')):.0f} mm:</b> the flight wanders slowly at about 0.1 Hz by 12–17 mm, which the feedback noise model does not produce. The {F['rmse']['roll']['bias']:+.1f}° roll and {F['rmse']['pitch']['bias']:+.1f}° pitch are a constant offset: that day's raw mocap attitude was levelled differently. Once the offset is removed, the scatter is {sd(F, 'roll'):.2f}° and {sd(F, 'pitch'):.2f}°.</li>
      <li><b>Arm:</b> the simulated arm follows the q2/q3 sweep more closely than the real one, because the friction model has a single level and no stick–slip. The simulated wrist rests {abs(A['rmse']['q4']['bias']):.1f}° from zero on the circle, where the real wrist stays at zero.</li>
    </ul>
  </section>
</div>

<div class=part>
  <div class=parthead><div class=no>2</div><h2>Whole-body controller tuned on the circle</h2>
    <p>Thirteen gains were tuned by CMA-ES on the flight-matched plant and gated on transport delay, a gust, the robustness plant and Isaac's clock. They are written to the hardware yaml and have not yet been flown.</p></div>

  <section>
    <div class=sechead><span class=sn>2.1</span><h3>Control parameters</h3></div>
    <p class=lead>Every parameter of the whole-body controller in DIRECT mode, read from <code>params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml</code>. Highlighted rows changed in the circle tune. <i>Before the circle tune</i> is the 2026-09-26 file: the flown controller plus the arm-channel fix and k<sub>ω</sub> 0.9.</p>
    {ctrl_table()}
    <ul class=notes>
      <li>Present in the file but inactive in this configuration: the GMO gains <code>wb_ko_t/r/q</code> (the observer is L1; <code>wb_use_gmo</code> is only its master switch), and <code>wb_l1_omega_i</code> and the <code>wb_l1_lc_var_*</code> priors, which belong to the 6-D attribution. SAFETY-mode takeoff and landing use the unchanged baseline position controller.</li>
    </ul>
  </section>

  <section>
    <div class=sechead><span class=sn>2.2</span><h3>Circle tracking RMSE before and after, hardware clock</h3></div>
    <p class=lead>This is the exact Python law (<code>controller.py</code> + <code>l1_observer.py</code>, parity-locked to the C++) flying the planner's own circle streams. It runs on the flight-matched plant at RTF 1, the clock the hardware runs on, with feedback noise calibrated to the flights' hover. Seeds 11–13 are pooled. <i>Before</i> is the gains before the circle tune; <i>after</i> is the final column of 2.1.</p>
    {track_table('bench', B)}
    <ul class=notes>
      <li><b>What improves:</b> end-effector position by {rng('ee_pos_norm')} on the mirror plant ({B['show']['ship']['ee_pos_norm']:.1f} → {B['show']['h1b']['ee_pos_norm']:.1f} mm on the showcase circle), CoM position by {rng('com_pos_norm')}, and attitude by {rng('att_norm')}.</li>
      <li><b>What it costs:</b> the velocity gain falls from 20 to 12.6 and the task stiffness rises to 212 N/m, so velocity error rises {rng('com_vel_norm')}, body-rate error {rng('rate_norm')} and joint-rate error {rng_many(['qd1', 'qd2', 'qd3', 'qd4'])}. Height error rises about 0.5 mm. None of these reaches a limit: saturation is 0 %, and peak arm torque is {max(B[c]['h1b']['_meta']['tau_pk'] for c in ('flown', 'show', 'show_rob')):.2f} of 3.0 N·m.</li>
      <li><b>Robustness plant</b> (<code>_sim_robustness</code>: allocator k<sub>f</sub> +17.6 %, mass and inertia ×1.10, CoM shift 10/10/5 mm, arm friction and mass ×1.05): all seeds complete, and end-effector error falls {B['show_rob']['ship']['ee_pos_norm']:.1f} → {B['show_rob']['h1b']['ee_pos_norm']:.1f} mm.</li>
    </ul>
  </section>

  <section>
    <div class=sechead><span class=sn>2.3</span><h3>Circle tracking RMSE before and after, Isaac Sim</h3></div>
    <p class=lead>One Isaac flight per cell, on the same plants and circles, at RTF 0.48. The node does not log the desired body rate, so the body-rate rows are missing here.</p>
    {track_table('isaac', I)}
    <div class=callout><b>Why attitude and velocity worsen in Isaac but not on the bench.</b> The controller's observer runs on the wall clock while Isaac's plant runs at half speed. The estimate then carries a term, −(1−RTF)·ṗ, that the hardware does not have. The bench run on Isaac's clock (seed 11, showcase circle) reproduces the Isaac flights. After the tune it gives attitude {ck['h1b']['att_norm']:.2f}° against Isaac's {I['show']['h1b']['att_norm']:.2f}°, a standing pitch of {ck['h1b']['_meta']['eR_mean_deg_model'][0]:.2f}° against {I['show']['h1b']['_meta']['eR_mean_deg_model'][0]:.2f}°, and CoM error {ck['h1b']['com_pos_norm']:.0f} against {I['show']['h1b']['com_pos_norm']:.0f} mm. Before the tune it gives {ck['ship']['att_norm']:.2f}° against {I['show']['ship']['att_norm']:.2f}° and {ck['ship']['com_pos_norm']:.0f} against {I['show']['ship']['com_pos_norm']:.0f} mm. On the hardware clock the same gains lower attitude error (2.2). Isaac's absolute errors, about 100 mm here against 12–19 mm on the bench, are dominated by the same coupling and are not a hardware prediction. What Isaac does confirm: all six flights complete with no saturation and no joint clamp, and end-effector error falls on every plant and circle, most on the robustness plant ({I['show_rob']['ship']['ee_pos_norm']:.0f} → {I['show_rob']['h1b']['ee_pos_norm']:.0f} mm).</div>
  </section>

  <section>
    <div class=sechead><span class=sn>2.4</span><h3>Circular trajectory and q2 sweep</h3></div>
    <p class=lead>The circle is a compatible whole-body trajectory from <code>fsc_trajectory_planner</code>. The end-effector flies a horizontal circle with its heading along the tangent. The arm's redundant degree of freedom, q2, follows an assigned sine at a fixed fold β, and the base and remaining joints are solved to match. The recommended showcase keeps the current defaults except for the bold entries.</p>
    <div class=tables2>{tb_circle}{tb_q2}</div>
    <ul class=notes>
      <li><b>Why these settings</b> (bench, final gains, 48 circles): lap time is the real lever (24 → 40 s lowers end-effector error 14.1 → 11.4 mm). Radius costs almost nothing (0.5 m and 0.75 m at a 24 s lap: 14.4 and 14.1 mm). Clockwise beats counter-clockwise by 14–21 %, because of the identified standing force. The arm sweep makes no measurable difference (five sweeps within 0.2 mm), so it is sized for visibility: ±15° over one lap.</li>
      <li>The 0.75 m, 24 s lap gives the requested ~0.2 m/s. The defaults in the yaml stay at the flown circle until the showcase is flown.</li>
    </ul>
  </section>
</div>

<footer>
  <span>Sources: <code>docs/docs_aerial_manipulator/sim2real_tuning_20260926/</code> (<code>tools/state_rmse.py</code>, <code>compare.py</code>, <code>fit_plant.py</code>) and <code>circle_tune_20260927/</code> (<code>tools/circle_states.py</code>, <code>circle_bench.py</code>, <code>tune_cma.py</code>). The earlier long-form report is <code>report_detailed.html</code> in the same folder.</span>
</footer>
</div>
<script>const FIGS={json.dumps(circ + step, separators=(',', ':'))};</script>
<script>{JS}</script>
"""
    open(out, "w").write(page)
    print(f"wrote {out} ({len(page) / 1e3:.0f} kB)")


if __name__ == "__main__":
    main()
