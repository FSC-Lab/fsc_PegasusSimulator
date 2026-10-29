#!/usr/bin/env python3
"""Write the BUILD PLAN's aircraft as an airframe file (2026-10-27).

    python3 scripts/plan_airframe.py [--xlsx ~/Downloads/VTOL_Building_Plan.xlsx] [--config A|B|C]
        [--mass parts|components] [--motor KV450] [--front 13x8] [--rear 13x5.5] [--volts 22.2]
        [--out <controller package>/airframe_plan_<config>.cfg]

The workbook (VTOL_Building_Plan.xlsx: Inputs, Components, Parts, Parts summary) is the source; the
output is an airframe_id.cfg-format file WITH a [layout] section, so the simulator builds its plant
from the same file every controller reads (fsc_aerial_manipulation.rotorcraft.vtol_utils
.airframe_vtol_config) -- one description of the aircraft, selected by FSC_AIRFRAME_ID on both sides.
Re-run it whenever the workbook changes (weighed masses, a new prop) and the aircraft follows.

What each section is made of, and how sure it is:
  [mass]     the Components tab's positions and local inertias with the PARTS-LEVEL masses: each
             component's mass scaled by its group's Parts-summary estimate (after the trims the
             workbook marks Apply = Y) over the mass model -- the plan's own "Re-baseline the mass
             model" step, done here. The avionics bundle is re-placed (the workbook's Solve mode) so
             the CG stays on the bay datum. `--mass components` takes the mass model as it stands.
             Config A carries the tank module DRY; the water is VTOL_TANK_KG (a mission setting, as
             on the CAD airframe) and the line to launch with is printed and written in the header.
  [rotors]   hubs = the nacelle points (P01-P04); the propulsion PREDICTED by scripts/rotor_trade.py's
             models (the bench-fitted 4010 motor line rewound to the plan's KV, the UIUC wind-tunnel
             data of the Aero-naut CAM carbon FOLDING blades the plan buys): static thrust and speed
             at full throttle, J0, torque/thrust and spin lag at the hover point -- front and rear
             separately (the rear_* keys). Replace with the thrust-rig session (plan task 14).
  [tilt]     the plan's spec (tau 0.0756 s, 106 deg/s); travel -20..+93 deg in the SIM convention
             (0 = thrust up): the plan's beta runs the other way (0 = forward), so its hard stops
             -3 / +115 deg are +93 / -25 here and its 0-110 deg corridor is +90..-20.
  [surfaces] the sim's servo (tau 0.03 s = the plan's H01 spec, 400 deg/s, 25 deg).
  [aero]     THE CAD AIRFRAME'S COEFFICIENTS on the plan's reference geometry (S, b, MAC from
             Inputs) -- a placeholder until the CFD; replace this section and only this section.
  [layout]   the plant's geometry no controller reads: surface stations from Components, boxes.
  [tuning]   controller gains retuned for this airframe, gated on scripts/airframe_ctrl_gate.sh
             (2026-10-28): the cascade's roll law (the 2.0 m wing's 2.6x roll damping halved its slow
             roll pole and rang the orbit) and the safe controller's attitude law (the predicted spin
             lag left it ~50 ms of delay margin). Gated on the PLACEHOLDER aero: after --aero (the CFD),
             re-run airframe_ctrl_gate.sh before trusting them.

Frames: the plan's x runs AFT from the nose with y left and z up; here x is FORWARD (FLU) from the
bay datum x_T, and z is from the CG (the hubs' z convention of airframe_id.cfg).
"""
import argparse
import contextlib
import copy
import datetime
import hashlib
import io
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import inner_dir, load_airframe          # noqa: E402

NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}
RID = "{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id"

# Components ID -> the Parts-summary mass group it belongs to (the workbook's own grouping: the
# summary's "mass model" column is exactly these rows' sums)
GROUP = {**{f"S{i:02d}": "Wing" for i in range(1, 6)}, "S06": "Fuselage",
         **{f"S{i:02d}": "Tail" for i in (7, 8, 9)}, "S10": "Booms", "S11": "Booms",
         **{f"S{i:02d}": "Gear" for i in (12, 13, 14)},
         **{f"P{i:02d}": "Nacelles" for i in range(1, 5)}, **{f"P{i:02d}": "Tilt drive" for i in range(5, 9)},
         **{f"E{i:02d}": "Electrical" for i in range(1, 4)}, **{f"H{i:02d}": "Servos & hardware" for i in range(1, 8)},
         **{f"A{i:02d}": "Avionics" for i in range(1, 6)}, "B01": "Battery",
         "M01": "Payload A", "M02": "Payload B", "M03": "Payload C"}
# a trim on the Parts summary -> the group it lightens (an unknown trim marked Y is an error, so a
# new trim in the workbook can never be silently ignored)
TRIM_GROUP = (("spar", "Wing"), ("ribs", "Wing"), ("boom power", "Electrical"), ("booms", "Booms"),
              ("surface servos", "Servos & hardware"), ("tilt yokes", "Nacelles"), ("tray", "Fuselage"),
              ("skids", "Gear"), ("tank", "Payload A"))


# ------------------------------------------------------------------------------------------------ xlsx
def read_xlsx(path):
    """{sheet: {cell: value}} from the cached values (stdlib only; the workbook must have been saved by
    Excel/LibreOffice so its formulas carry results)."""
    z = zipfile.ZipFile(path)
    ss = []
    if "xl/sharedStrings.xml" in z.namelist():
        for si in ET.fromstring(z.read("xl/sharedStrings.xml")).findall("m:si", NS):
            ss.append("".join(t.text or "" for t in si.iter("{%s}t" % NS["m"])))
    wb = ET.fromstring(z.read("xl/workbook.xml"))
    rels = {r.get("Id"): r.get("Target") for r in ET.fromstring(z.read("xl/_rels/workbook.xml.rels"))}
    out = {}
    for sh in wb.find("m:sheets", NS):
        tgt = rels[sh.get(RID)]
        tgt = tgt if tgt.startswith("xl/") else "xl/" + tgt.lstrip("/")
        cells = {}
        for c in ET.fromstring(z.read(tgt)).iter("{%s}c" % NS["m"]):
            v = c.find("m:v", NS); t = c.get("t")
            if t == "inlineStr":
                val = "".join(x.text or "" for x in c.iter("{%s}t" % NS["m"]))
            elif v is None or v.text is None:
                continue
            elif t == "s":
                val = ss[int(v.text)]
            else:
                try:
                    val = float(v.text)
                except ValueError:
                    val = v.text
            cells[c.get("r")] = val
        out[sh.get("name")] = cells
    return out


def rows_of(cells):
    """{row number: {column letters: value}}."""
    rows = {}
    for ref, v in cells.items():
        m = re.match(r"([A-Z]+)(\d+)$", ref)
        rows.setdefault(int(m.group(2)), {})[m.group(1)] = v
    return rows


def num(v):
    return float(v) if isinstance(v, (int, float)) else float("nan")


# ------------------------------------------------------------------------------------------------ mass
def components(wb):
    rows = rows_of(wb["Components"])
    hdr = next(r for r, c in rows.items() if c.get("A") == "ID")
    out = []
    for r in sorted(k for k in rows if k > hdr):
        c = rows[r]
        cid = c.get("A")
        if not isinstance(cid, str) or not re.match(r"^[A-Z]\d\d$", cid):
            continue
        out.append(dict(id=cid, name=c.get("C", ""), m=num(c.get("E")) / 1000.0,
                        p=np.array([num(c.get("H")), num(c.get("I")), num(c.get("J"))]),
                        inc=[int(num(c.get(k, 0))) for k in ("O", "P", "Q")],
                        Il=np.array([num(c.get("S", 0.0)), num(c.get("T", 0.0)), num(c.get("U", 0.0))])))
    return out


def parts_summary(wb):
    """({group: parts estimate g}, {group: model g}, [(trim, saving g, group, applied)])."""
    rows = rows_of(wb["Parts summary"])
    hdr = next(r for r, c in rows.items() if c.get("A") == "Mass group")
    est, model = {}, {}
    for r in range(hdr + 1, hdr + 40):
        c = rows.get(r, {})
        a = c.get("A")
        if not isinstance(a, str) or a.startswith("Core aircraft") or a.startswith("Slosh emulator"):
            break
        est[a] = num(c.get("C")); model[a] = num(c.get("B"))
    th = next(r for r, c in rows.items() if c.get("A") == "Trim")
    trims = []
    for r in range(th + 1, th + 40):
        c = rows.get(r, {})
        a = c.get("A")
        if not isinstance(a, str) or a.startswith("Applied"):
            break
        grp = next((g for key, g in TRIM_GROUP if key in a.lower()), None)
        applied = str(c.get("D", "")).strip().upper() == "Y"
        if applied and grp is None:
            raise SystemExit(f"plan_airframe: the workbook applies a trim this script cannot place: {a!r}")
        trims.append((a, num(c.get("B")), grp, applied))
    return est, model, trims


def module_dry_g(wb, letter):
    """Parts tab: the flying mass of payload module `letter` without any water ('Supply' lines)."""
    rows = rows_of(wb["Parts"])
    tot = water = 0.0
    for c in rows.values():
        pid = c.get("A")
        if isinstance(pid, str) and pid.startswith("P" + letter) and str(c.get("L", "")).strip() == "Y":
            g = num(c.get("H"))
            tot += g
            if "water" in str(c.get("C", "")).lower():
                water += g
    return tot, water


def mass_properties(comp, cfg, scale, x_T, avionics_solve=True):
    """(M, CG, I about the CG, avionics x) in the PLAN frame, config `cfg`, masses x scale[group]."""
    k = "ABC".index(cfg)
    sel = []
    for q in comp:
        if not q["inc"][k] or q["m"] <= 0.0:
            continue
        f = scale.get(GROUP.get(q["id"]), 1.0)
        sel.append(dict(q, m=q["m"] * f, Il=q["Il"] * f))
    x_av = None
    if avionics_solve:                    # the workbook's Solve mode: the bundle holds the CG on x_T
        av = [q for q in sel if q["id"].startswith("A")]
        rest = [q for q in sel if not q["id"].startswith("A")]
        m_av = sum(q["m"] for q in av)
        x_av = x_T - sum(q["m"] * (q["p"][0] - x_T) for q in rest) / m_av
        for q in av:
            q["p"] = np.array([x_av, q["p"][1], q["p"][2]])
    M = sum(q["m"] for q in sel)
    cg = sum(q["m"] * q["p"] for q in sel) / M
    I = np.zeros((3, 3))
    for q in sel:
        r = q["p"] - cg
        I += np.diag(q["Il"]) + q["m"] * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    return M, cg, I, x_av, sel


# ------------------------------------------------------------------------------------------------ propulsion
def propulsion(motor, front, rear, volts, hover_front, hover_rear):
    """Per-pair rotor parameters from the rotor-trade models. A prop name 'DxP' is the UIUC Aero-naut
    CAM carbon folding blade of that size; a pitch between two measured ones is interpolated."""
    import rotor_trade as RT
    db = os.path.expanduser("~/data/uiuc-propDB/UIUC-propDB")
    with contextlib.redirect_stdout(io.StringIO()):
        uiuc = RT.load_uiuc(db)
        props, _, _ = RT.build_candidates(db)
        ms, bench = RT.motors()
        RT.calibrate_lag(ms, props, bench)
    mm = re.match(r"^KV(\d+)$", motor)
    if mm:                                    # the 4010 frame rewound to this KV (R ~ 1/KV^2)
        base = next(m for m in ms if m.name == "MN4010 KV475")
        kv = float(mm.group(1))
        mot = copy.copy(base); mot.name = f"4010 {motor} (MN4010 KV475 rewound)"
        mot.kv = kv; mot.kv_si = kv * 2 * math.pi / 60; mot.Kt = 1 / mot.kv_si
        mot.s = (0.098 * (475.0 / kv) ** 2 + bench["rr10"]) * mot.kv_si ** 2; mot.R_sys = mot.s * mot.Kt ** 2
    else:
        mot = next(m for m in ms if m.name == motor)
    mot.V = volts

    def blade(name):
        d, p = (float(v) for v in name.lower().split("x"))
        have = sorted(pp for (fam, dd, pp) in uiuc if fam == "ancf" and dd == d)
        if p in have:
            q = copy.copy(uiuc[("ancf", d, p)]); q.mass_g = 28.0 * (d / 13.0) ** 2.3
            return [(1.0, q)]
        lo = max(v for v in have if v < p); hi = min(v for v in have if v > p)
        w = (p - lo) / (hi - lo)
        out = []
        for ww, pp in ((1.0 - w, lo), (w, hi)):
            q = copy.copy(uiuc[("ancf", d, pp)]); q.mass_g = 28.0 * (d / 13.0) ** 2.3
            out.append((ww, q))
        return out

    def pair(name, T_hover):
        acc = dict(max_thrust=0.0, omega_max=0.0, prop_j0=0.0, cq=0.0, tau_spin=0.0)
        for w, q in blade(name):
            om = RT.omega_full(mot, q, 0.0)
            T, Q, _ = RT.forces(q, om, 0.0)
            pt = RT.point(mot, q, T_hover, 0.0)
            acc["max_thrust"] += w * T; acc["omega_max"] += w * om; acc["prop_j0"] += w * q.j0()
            acc["cq"] += w * pt["Q"] / pt["T"]; acc["tau_spin"] += w * RT.tau_hover(mot, q, pt, q.mass_g)
        acc["prop_d"] = float(name.lower().split("x")[0]) * 0.0254
        return acc

    return mot.name, pair(front, hover_front), pair(rear, hover_rear)


# ------------------------------------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--xlsx", default=os.path.expanduser("~/Downloads/VTOL_Building_Plan.xlsx"))
    ap.add_argument("--config", default="A", choices=("A", "B", "C"))
    ap.add_argument("--mass", default="parts", choices=("parts", "components"))
    ap.add_argument("--motor", default="KV450", help="KV<n> (the 4010 frame rewound) or a rotor_trade motor name")
    ap.add_argument("--front", default="13x8"); ap.add_argument("--rear", default="13x5.5")
    ap.add_argument("--volts", type=float, default=22.2)
    ap.add_argument("--aero", default=None,
                    help="a file whose [aero] section replaces the placeholder (the CFD's); its s_wing / b_wing / "
                         "c_wing are used as given. Default: the CAD airframe's coefficients on the plan's S, b, MAC")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(inner_dir(), f"airframe_plan_{a.config}.cfg")

    wb = read_xlsx(a.xlsx)
    inp = {c.get("A"): c.get("B") for c in rows_of(wb["Inputs"]).values() if isinstance(c.get("A"), str)}

    def inp_(prefix):
        return num(next(v for k, v in inp.items() if k.startswith(prefix)))
    S, b = inp_("Wing reference area"), inp_("Wing span")
    mac, x_le = inp_("Mean aerodynamic chord"), inp_("MAC leading edge")
    x_T = inp_("Bay datum x_T")

    comp = components(wb)
    est, model, trims = parts_summary(wb)
    scale, cut = {}, {}
    if a.mass == "parts":
        for name, sav, grp, applied in trims:
            if applied:
                cut[grp] = cut.get(grp, 0.0) + sav
        for g, m0 in model.items():
            scale[g] = (est[g] - cut.get(g, 0.0)) / m0 if m0 > 0 else 1.0
    water_kg = 0.0
    if a.config == "A":
        tot, water = module_dry_g(wb, "A")
        # the module's Parts estimate (after any tank trim) without its water: the airframe carries the
        # tank, the cells and the frame; the water is VTOL_TANK_KG
        mod_g = (tot - cut.get("Payload A", 0.0)) if a.mass == "parts" else model["Payload A"]
        water_kg = water / 1000.0
        dry_g = mod_g - water
        m01 = next(q for q in comp if q["id"] == "M01")
        scale["Payload A"] = dry_g / (m01["m"] * 1000.0)
    M, cg, I, x_av, sel = mass_properties(comp, a.config, scale, x_T)
    # the workbook's own layout check: the avionics bundle must lie wholly ahead of the bay's front
    # bulkhead or wholly behind its aft one. Solve mode can put it INSIDE the bay when the masses move
    # (it did, once the Parts-level masses went in): say so, and what the CG is with the bundle at the
    # nearest legal station behind the bay.
    bay_f, bay_a, half = inp_("Bay front bulkhead, x"), inp_("Bay aft bulkhead, x"), inp_("Avionics bundle half-length")
    av_warn = ""
    if x_av - half < bay_a and x_av + half > bay_f:
        x_leg = bay_a + half
        m_av = sum(q["m"] for q in sel if q["id"].startswith("A"))
        cg_leg = cg[0] + m_av * (x_leg - x_av) / M
        m_wet = M + water_kg
        cg_leg_wet = (cg_leg * M + x_T * water_kg) / m_wet
        av_warn = (f"the avionics bundle solved to x {x_av:.3f} m sits INSIDE the payload bay ({bay_f:g}-{bay_a:g}); "
                   f"behind the bay (x {x_leg:.3f}) the CG would be {1000 * (cg_leg_wet - x_T):+.1f} mm from the datum "
                   f"({100 * (cg_leg_wet - x_le) / mac:.1f}% MAC) -- the workbook's 'Avionics bundle clear of bay' check")
        print("  WARNING " + av_warn)

    # the tank (config A): the water's own centre, from the module box (its floor + 1.5 mm wall + depth/2)
    dz_tank = 0.0
    if a.config == "A":
        q = next(q for q in sel if q["id"] == "M01")
        row = next(c for c in rows_of(wb["Components"]).values() if c.get("A") == "M01")
        box_h = num(row.get("N"))
        floor = q["p"][2] - box_h / 2.0 + 0.0015
        depth = water_kg / 1000.0 / (0.207 * 0.192)              # the 1.5 mm-walled 210 x 195 tank
        z_w = floor + depth / 2.0
        dz_tank = z_w - cg[2]

    hubs_plan = {q["id"]: q["p"] for q in sel if q["id"] in ("P01", "P02", "P03", "P04")}
    hubs = [np.array([x_T - hubs_plan[k][0], hubs_plan[k][1], hubs_plan[k][2] - cg[2]]) for k in ("P01", "P02", "P03", "P04")]
    m_tot = M + water_kg
    # hover split with the water folded (the tank sits on the datum, the CG on the datum: equal pairs)
    xf, xr = hubs[0][0] - (x_T - cg[0]), -(hubs[2][0] - (x_T - cg[0]))
    T_f = m_tot * 9.81 * xr / (xf + xr) / 2.0; T_r = m_tot * 9.81 * xf / (xf + xr) / 2.0
    motor_name, F, R = propulsion(a.motor, a.front, a.rear, a.volts, T_f, T_r)

    base = load_airframe(os.path.join(inner_dir(), "airframe_id.cfg"))    # the CAD airframe's aero
    aero_src = None
    if a.aero:                                                               # the CFD's [aero], key for key
        import configparser
        cp = configparser.ConfigParser(inline_comment_prefixes=("#",), interpolation=None)
        cp.optionxform = str
        cp.read(a.aero)
        aero_src = {k: float(v) for k, v in cp.items("aero")}
    lay = {}
    for key, cid in (("aileron_l", "H01"), ("aileron_r", "H02"), ("elevator", "S07"), ("rudder", "S08")):
        q = next(q for q in comp if q["id"] == cid)
        lay[key] = (x_T - q["p"][0], q["p"][1], q["p"][2] - cg[2])
    gear_bottom = min(q["p"][2] for q in sel if q["id"] in ("S12", "S13", "S14")) - 0.025   # 50 mm wheels
    h_box = 2.0 * (cg[2] - gear_bottom)

    with open(a.xlsx, "rb") as f:
        wb_md5 = hashlib.md5(f.read()).hexdigest()
    stamp = datetime.datetime.fromtimestamp(os.path.getmtime(a.xlsx)).strftime("%Y-%m-%d %H:%M")
    h_pct = 100.0 * (cg[0] - x_le) / mac
    f6 = lambda v: f"{v:.6f}".rstrip("0").rstrip(".") if abs(v) > 1e-12 else "0.0"   # noqa: E731
    vec = lambda v: ", ".join(f6(float(x)) for x in v)                                # noqa: E731
    tank_line = (f"VTOL_TANK_KG={water_kg:g} VTOL_TANK_R=0.10 VTOL_TANK_F1=1.10 VTOL_TANK_ZETA=0.006 "
                 f"VTOL_TANK_DX=0 VTOL_TANK_DZ={dz_tank:+.3f}") if a.config == "A" else "VTOL_TANK unset (no water)"
    txt = f"""# THE BUILD PLAN'S AIRCRAFT, configuration {a.config} -- written by scripts/plan_airframe.py; do not hand-edit
# the [mass] / [rotors] / [layout] sections: edit the workbook and re-run it. The [aero] section is
# the one to replace (the CFD, due ~2026-11-27); everything reads it from here.
#   source   {a.xlsx}  (saved {stamp}, md5 {wb_md5})
#   written  {datetime.datetime.now().strftime("%Y-%m-%d %H:%M")}  --config {a.config} --mass {a.mass} --motor {a.motor} --front {a.front} --rear {a.rear} --volts {a.volts:g}
#   mass     {a.mass}: {"Parts-level estimate after the workbook's applied trims, on the Components positions" if a.mass == "parts" else "the Components mass model as it stands"};
#            avionics re-placed at x {x_av:.4f} m (plan frame) to hold the CG on the datum x_T {x_T:g}
#            dry {M:.4f} kg + water {water_kg:g} kg = {m_tot:.4f} kg; CG {h_pct:.1f}% MAC, {1000 * cg[2]:.1f} mm above the plan's z datum
#   layout   {"WARNING: " + av_warn if av_warn else "the workbook's layout checks hold"}
#   rotors   {motor_name} at {a.volts:g} V; front {a.front}, rear {a.rear} (Aero-naut CAM carbon folding, UIUC
#            wind tunnel); PREDICTED by scripts/rotor_trade.py's models -- replace with the thrust rig
#   aero     {("[aero] of " + a.aero) if a.aero else f"the CAD airframe's coefficients (airframe_id.cfg) on the plan's S {S:g} m^2, b {b:g} m, MAC {mac:g} m -- a placeholder until the CFD (--aero <file>)"}
#   tilt     sim convention (0 = thrust up): the plan's beta = 90 - this; its hard stops -3 / +115 = +93 / -25
#   launch   FSC_AIRFRAME_ID={out}
#            {tank_line}
#   frames   x forward (FLU) from the bay datum, z from the CG (the plan's x runs aft from the nose)

[mass]
airframe_kg = {M:.6f}      # no water ({"config A: the tank module dry" if a.config == "A" else "config " + a.config})
cg_x = {x_T - cg[0]:.6f}            # CG forward of the datum [m]
ixx = {I[0, 0]:.6f}                 # about the CG [kg m^2] (the plan's I_xz {-I[0, 2]:+.5f} in FLU is not carried)
iyy = {I[1, 1]:.6f}
izz = {I[2, 2]:.6f}

[rotors]
max_thrust = {F['max_thrust']:.3f}          # front, static at throttle 1 [N]  PREDICTED
hub_fl = {vec(hubs[0])}
hub_fr = {vec(hubs[1])}
hub_rl = {vec(hubs[2])}
hub_rr = {vec(hubs[3])}
spin = 1, -1, -1, 1
tiltable = 1, 1, 0, 0
cq = {F['cq']:.5f}                 # front: torque / thrust at the hover point [m]
prop_d = {F['prop_d']:.4f}             # {a.front}
prop_j0 = {F['prop_j0']:.3f}
omega_max = {F['omega_max']:.1f}           # front, static at throttle 1 [rad/s]
tau_spin = {F['tau_spin']:.4f}            # front, at its hover point [s]
rear_max_thrust = {R['max_thrust']:.3f}     # the rear pair: {a.rear}
rear_cq = {R['cq']:.5f}
rear_prop_d = {R['prop_d']:.4f}
rear_prop_j0 = {R['prop_j0']:.3f}
rear_omega_max = {R['omega_max']:.1f}
rear_tau_spin = {R['tau_spin']:.4f}

[tilt]
tau = 0.0756                # the plan's spec (13 rad/s)
rate_deg_s = 106.0          # the plan's spec
min_deg = -20.0             # the plan's beta 110 deg (aft)
max_deg = 93.0              # the plan's -3 deg hard stop

[surfaces]
tau = 0.03
rate_deg_s = 400.0
max_deflection_deg = 25.0

[aero]
"""
    for k in ("rho", "s_wing", "b_wing", "c_wing", "cl0", "cla", "clq", "clde", "cd0", "kind", "cdb2", "cyb",
              "cydr", "cy_max", "clb", "clda", "clp", "clr", "cldr", "cm0", "cma", "cmq", "cmde", "cnb", "cndr",
              "cnr", "cnp", "cnda", "stall_deg", "beta_break_deg", "blend_m", "min_v"):
        if aero_src is not None:
            if k not in aero_src:
                raise SystemExit(f"plan_airframe: {a.aero} [aero] has no {k!r}")
            v, tag = aero_src[k], ""
        else:
            v = {"s_wing": S, "b_wing": b, "c_wing": mac}.get(k, getattr(base.aero, k))
            tag = "   # the plan's (Inputs)" if k in ("s_wing", "b_wing", "c_wing") else ""
        txt += f"{k} = {f6(float(v))}{tag}\n"
    txt += f"""
[layout]
name = plan_vtol_{a.config}
spin_offset = 0.0, 0.0, 0.05
aileron_l = {vec(lay['aileron_l'])}
aileron_r = {vec(lay['aileron_r'])}
elevator = {vec(lay['elevator'])}
rudder = {vec(lay['rudder'])}
surface_aileron_size = 0.057, 0.38, 0.01
surface_elevator_size = 0.04, 0.637, 0.01
surface_rudder_size = 0.06, 0.01, 0.30
body_size = 1.0, 0.21, {h_box:.3f}    # the collider: its floor is the wheels' bottom
collision_size = 1.0, 0.21, {h_box:.3f}
link_mass = 0.05

[tuning]
# gated 2026-10-28 on the placeholder aero (scripts/airframe_ctrl_gate.sh, results ~/flight_bags/ctrl_gate);
# re-gate after the CFD. A key left out = that node's shipped default.
cascade_kR_roll = 6.0    # 3.0 shipped: slow roll pole -0.56 -> -1.03 rad/s at 16 m/s (the CAD's -1.17); orbit +-14 m -> +-0.1
cascade_kW_roll = 1.8    # 1.2 shipped: keeps the hover roll loop at zeta 0.59 with the stiffer kR
safe_kR = 5.0            # 9.0 shipped: transport-delay margin ~50 -> ~100 ms on the predicted spin lag
safe_kW = 1.5            # 1.8 shipped; 6 m/s crosswind hold 0.02 -> 0.19 m (clip-only 0.37 m)
fsc_attctl_kp_angle_z = 1.5   # 6.0 shipped (the C++ autopilot's yaw angle): PX4's tilt-yaw rate loop here is ~1.6 rad/s
                              # (6.4 rad/s^2 per unit at MC_YAWRATE_P 0.25); at 6 the heading rang +-25 deg at 0.2 Hz (HIL)
"""
    with open(out, "w") as f:
        f.write(txt)
    tw_f = F["max_thrust"] / T_f; tw_r = R["max_thrust"] / T_r
    print(f"wrote {out}")
    print(f"  config {a.config} ({a.mass}): dry {M:.3f} kg + water {water_kg:g} = {m_tot:.3f} kg; CG {h_pct:.1f}% MAC;"
          f" I {I[0, 0]:.3f} / {I[1, 1]:.3f} / {I[2, 2]:.3f}; avionics x {x_av:.3f}")
    print(f"  hubs (FLU from the datum, z from the CG): front {vec(hubs[0])}  rear {vec(hubs[2])}")
    print(f"  rotors ({motor_name}, {a.volts:g} V): front {a.front} {F['max_thrust']:.1f} N static ({tw_f:.2f}x its hover {T_f:.2f} N),"
          f" J0 {F['prop_j0']:.2f}, cq {F['cq']:.4f}, tau {F['tau_spin']:.3f} s | rear {a.rear} {R['max_thrust']:.1f} N ({tw_r:.2f}x),"
          f" J0 {R['prop_j0']:.2f}, cq {R['cq']:.4f}, tau {R['tau_spin']:.3f} s")
    print(f"  launch: FSC_AIRFRAME_ID={out} {tank_line}")
    if a.aero:
        print("  !! [tuning] was gated on the placeholder aero: re-run scripts/airframe_ctrl_gate.sh on this file")


if __name__ == "__main__":
    main()
