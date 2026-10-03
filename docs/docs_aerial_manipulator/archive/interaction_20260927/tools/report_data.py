"""Collect the report's plotted series into analysis/report_data.json."""
import glob, json, os, numpy as np
H = os.path.dirname(os.path.abspath(__file__)); D = os.path.join(H, "..", "data"); A = os.path.join(H, "..", "analysis")
def tr(nm): return np.load(os.path.join(D, f"trace_{nm}.npz"), allow_pickle=True)
def ds(t, y, t0, t1, hz=10):
    m = (t >= t0) & (t <= t1); t = t[m]; y = y[m]
    k = max(1, int(round(1 / (hz * np.median(np.diff(t))))))
    return [[round(float(a) - t0, 2), round(float(b), 4)] for a, b in zip(t[::k], y[::k])]
out = {}
# Fig 1: 3 N radial push (model +y), rendered force and EE task deflection
f1 = {}
for nm, lab in (("force_free", "shipped, free"), ("force_thr1_slow", "shipped, 1 N threshold"),
                ("force_thr2_fast", "reading 2 rad/s, 2 N threshold"), ("force_task_fast", "reading 2 rad/s, task flag")):
    d = tr(nm); t = d["t"]
    cons = d["H3_F_hat_f"][:, 1] * d["H_chi"]
    ey = (d["H3_e_ee"][:, 1] - d["H3_e_x"][:, 1]) * 1e3
    f1[lab] = {"F": ds(t, cons, 12, 42), "ey": ds(t, ey, 12, 42), "chi": ds(t, d["H_chi"], 12, 42)}
    if nm == "force_free":
        f1["_true"] = ds(t, d["H3_F_true"][:, 1], 12, 42)
out["push3"] = f1
# Fig 2: 100 g placement, desk force (F_true_z + m g) with a 25 s press
f2 = {}
for nm, lab in (("place_free", "shipped, free"), ("place_grip_slow", "shipped reading, gripper flag"),
                ("place_grip_fast", "reading 2 rad/s, gripper flag")):
    d = tr(nm); t = d["t"]; mk = dict(zip(d["marks"][0], d["marks"][1]))
    t0 = float(mk["t_place0"]) - 1
    z = d["H3_F_true"][:, 2] + 0.1 * 9.81
    # desk force only while attached and pressing
    f2[lab] = ds(t, z, t0, float(mk["t_open"]) - 0.1)
out["place100"] = f2
out["place100_target"] = 0.1 * 9.81 + 211.9 * 0.01
# Fig 3: hardware reading noise floor vs omega_x
dt = 1 / 250; nf = {}
for w in (0.207, 0.5, 1.0, 2.0, 5.0):
    L = []
    for f in sorted(glob.glob(os.path.join(D, "hw_*.npz"))):
        z = np.load(f); Aa = z["A"]; t = z["t"]; idx = np.where(Aa[:, 0] == 1)[0]
        a = np.exp(-w * dt); y = np.zeros(3); o = np.full(len(t), np.nan)
        for i in idx:
            y = a * y + (1 - a) * np.nan_to_num(Aa[i, 97:100]); o[i] = np.linalg.norm(y)
        keep = (Aa[:, 0] == 1) & (t > t[idx[0]] + 10) & (t < t[idx[-1]] - 2)
        L.append(o[keep])
    v = np.concatenate(L)
    nf[str(w)] = {p: round(float(np.percentile(v, q)), 3) for p, q in (("p95", 95), ("p99", 99), ("p999", 99.9))} | {"max": round(float(v.max()), 3)}
out["noise"] = nf
# capacity sweeps
def load(n): return json.load(open(os.path.join(A, n)))
cap = load("sweep_capacity.json")
fc = {}
for r in cap:
    if r["scn"] == "force" and r["kw"].get("hw", True):
        k = r["kw"]["dir"]; fc.setdefault(k, []).append({"F": r["kw"]["F"], "ok": r["verdict"] == "completed",
            "tilt": round(r.get("tilt_pk", float("nan")), 2), "util": round(r.get("tau_util", float("nan")), 3),
            "cap": round(r.get("cap_pct", float("nan")), 1), "ex": round(r.get("ex_pk", float("nan")) * 1e3)})
out["force_cap"] = fc
pc = []
for r in cap:
    if r["scn"] == "payload" and r["chi"] == "gripper":
        pc.append({"m": r["kw"]["m"], "hw": r["kw"].get("hw", True), "util": round(r.get("tau_util", float("nan")), 3),
                   "carry_rms": round(r.get("ee_carry_rms", float("nan")) * 1e3, 1), "sag": round(-r.get("ee_carry_z", float("nan")) * 1e3, 1),
                   "tilt": round(r.get("tilt_pk", float("nan")), 1), "ex": round(r.get("ex_pk", float("nan")) * 1e3),
                   "place": round(r.get("place_force", float("nan")), 2), "tau": r.get("tau_pk")})
out["payload_cap"] = pc
bc = []
for r in cap:
    if r["scn"] == "box":
        bc.append({"F": r["kw"]["F"], "dir": r["kw"]["dir"], "ok": r["verdict"] == "completed",
                   "moved": round(r.get("box_moved", float("nan")) * 1e3), "tilt": round(r.get("tilt_pk", float("nan")), 1),
                   "ex": round(r.get("ex_push_pk", float("nan")) * 1e3) if r.get("ex_push_pk") == r.get("ex_push_pk") else None,
                   "util": round(r.get("tau_util", float("nan")), 2), "cap": round(r.get("cap_pct", float("nan")), 1),
                   "fc": round(r.get("fc_push_mean", float("nan")), 2)})
out["box_cap"] = bc
# base-ff what-if
bf = {}
for fn, tag in (("sweep_baseff.json", None), ("sweep_baseff_rot.json", "rot")):
    for r in load(fn):
        if r["scn"] == "payload" and r["chi"] == "gripper" and not r["kw"].get("unload") and r["kw"].get("place_hold", 8) == 8:
            key = tag or ("full" if r["kw"].get("base_ff") else "off")
            bf.setdefault(key, {})[str(r["kw"]["m"])] = {k: round(r[k] * 1e3) for k in ("ex_lift_pk", "ex_place_pk", "ex_release_pk")}
out["baseff"] = bf
json.dump(out, open(os.path.join(A, "report_data.json"), "w"))
print(json.dumps({k: (list(v)[:6] if isinstance(v, dict) else len(v) if isinstance(v, list) else v) for k, v in out.items()}, indent=0)[:1500])
print(json.dumps(out["noise"]))
print(json.dumps(out["baseff"]))
# Isaac flights (scored by score_isaac.py)
isa = []
for tag, name in (("T2", "shipped (H1b)"), ("T1", "2 rad/s + 2 N threshold"), ("T3", "2 rad/s + static contact")):
    p = os.path.join(D, f"int_{tag}_score.json")
    if os.path.exists(p):
        isa.append({"name": name, "tag": tag, "segs": json.load(open(p))})
out["isaac"] = isa
out["isaac_note"] = ("Deflection is the CoM-anchored end-effector task error along the force, averaged over the last 3 s of each hold; "
                     "the value in brackets is F/K<sub>y</sub>. Joint 3's working range ends at +50°. T1's landing was cut short "
                     "when the next flight was started; its force profile had completed.")
json.dump(out, open(os.path.join(A, "report_data.json"), "w"))
print("isaac flights:", [f["tag"] for f in isa])
