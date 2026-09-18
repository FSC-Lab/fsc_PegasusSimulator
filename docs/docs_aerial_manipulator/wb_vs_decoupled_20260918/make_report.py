#!/usr/bin/env python3
"""Build the flight-test-preparation report page (HTML artifact) from the
campaign's npz files: runs the scorer, embeds the plots, writes report.html."""
import base64
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PEG = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SCORER = os.path.join(PEG, "application", "robotic_arm", "utils", "am_ee_compare_score.py")
PY = "/usr/bin/python3"
ENV = dict(os.environ, PYTHONNOUSERSITE="1")


def score(pairs, prefix):
    cmd = [PY, SCORER, "--json", prefix + ".json", "--plot-prefix", prefix]
    for label, f in pairs:
        if os.path.exists(os.path.join(HERE, f)):
            cmd += ["--run", label, os.path.join(HERE, f)]
    out = subprocess.run(cmd, cwd=HERE, env=ENV, capture_output=True, text=True)
    if out.returncode != 0:
        print(out.stdout, out.stderr, file=sys.stderr)
    with open(prefix + ".json") as fh:
        return json.load(fh)


def img(path):
    if not os.path.exists(path):
        return ""
    with open(path, "rb") as fh:
        b = base64.b64encode(fh.read()).decode()
    return f'<img src="data:image/png;base64,{b}" alt="{os.path.basename(path)}">'


def f1(v, d=1):
    return "—" if v is None or isinstance(v, str) else f"{v:.{d}f}"


def compare_table(rows, cols):
    keys = [("ee_pos_mean_mm", "EE position, mean [mm]", 0), ("ee_pos_p95_mm", "EE position, p95 [mm]", 0),
            ("ee_lag_s", "EE lag fit [s]", 2), ("ee_residual_after_lag_mm", "EE residual after lag [mm]", 0),
            ("ee_head_mean_deg", "EE heading, mean [°]", 1), ("ee_head_p95_deg", "EE heading, p95 [°]", 1),
            ("base_pos_mean_mm", "Base position, mean [mm]", 0), ("base_pos_max_mm", "Base position, max [mm]", 0),
            ("base_vel_mean_mps", "Base velocity, mean [m/s]", 3), ("base_yaw_mean_deg", "Base yaw, mean [°]", 1),
            ("base_yaw_max_deg", "Base yaw, max [°]", 1), ("base_z_mean_mm", "Base altitude, mean [mm]", 1),
            ("tilt_max_deg", "Tilt, max [°]", 1), ("joint_rms_all_deg", "Joints rms [°]", 2),
            ("ee_ref_speed_mps", "EE reference speed [m/s]", 3), ("T_lap", "Lap time [s]", 0)]
    h = ["<table><thead><tr><th>metric</th>" + "".join(f"<th>{c}</th>" for c in cols) + "</tr></thead><tbody>"]
    for k, name, d in keys:
        h.append(f"<tr><td>{name}</td>" + "".join(f"<td>{f1(r.get(k), d)}</td>" for r in rows) + "</tr>")
    h.append("</tbody></table>")
    return "\n".join(h)


def main():
    circ = score([("Whole-Body", "wb_wb_c48.npz"), ("Decoupled", "decoupled_dec_c48.npz")], os.path.join(HERE, "cmp_c48"))
    fig8 = score([("Whole-Body", "wb_wb_f67_y135.npz"), ("Decoupled", "decoupled_dec_f67_y135.npz")], os.path.join(HERE, "cmp_f67"))
    fig8d = score([("Whole-Body", "wb_wb_f67_y90.npz"), ("Decoupled", "decoupled_dec_f67_y90.npz")], os.path.join(HERE, "cmp_f67d"))
    sweep = score([("circle lap 24 s", "wb_wb_c24.npz"), ("circle lap 32 s", "wb_wb_c32.npz"),
                   ("circle lap 48 s", "wb_wb_c48.npz"), ("fig-8 axis −45°", "wb_wb_f67.npz"),
                   ("fig-8 axis +45°", "wb_wb_f67_y90.npz"), ("fig-8 axis on x", "wb_wb_f67_y45.npz"),
                   ("fig-8 axis on y", "wb_wb_f67_y135.npz")], os.path.join(HERE, "sweep"))
    fast = score([("Whole-Body", "wb_wb_c24.npz"), ("Decoupled", "decoupled_dec_c24.npz")], os.path.join(HERE, "cmp_c24"))
    tune = score([("shipped ω_c,t 2.0", "wb_wb_c48.npz"), ("candidate 1.0", "wb_c48_wct1.npz")],
                 os.path.join(HERE, "cmp_tune"))
    tmpl = open(os.path.join(HERE, "report_template.html")).read()
    def g(rows, i, k, d=0):
        try:
            return f1(rows[i].get(k), d)
        except IndexError:
            return "—"
    page = (tmpl.replace("{{CIRCLE_TABLE}}", compare_table(circ, [r["label"] for r in circ]))
            .replace("{{FIG8D_TABLE}}", compare_table(fig8d, [r["label"] for r in fig8d]))
            .replace("{{C48_WB}}", g(circ, 0, "ee_pos_mean_mm")).replace("{{C48_DEC}}", g(circ, 1, "ee_pos_mean_mm"))
            .replace("{{F67_WB}}", g(fig8, 0, "ee_pos_mean_mm")).replace("{{F67_DEC}}", g(fig8, 1, "ee_pos_mean_mm"))
            .replace("{{LAG}}", "1.1–1.5")
            .replace("{{ORIENT_TEXT}}", "On world y (heading 135°) the whole-body run completed at 92 mm mean, "
                     "indistinguishable from the diagonals. On world x — the arm's axis, the poorly damped one of §1 — "
                     "the whole-body rig's tilt grew from 5° to 20° over the last third of the run and its tilt "
                     "watchdog reverted it to SAFETY at 113 s of 138 s. <b>Fly the figure-8 with its long axis on "
                     "world y: take off heading 135°.</b>")
            .replace("{{FIG8_TABLE}}", compare_table(fig8, [r["label"] for r in fig8]))
            .replace("{{FAST_TABLE}}", compare_table(fast, [r["label"] for r in fast]))
            .replace("{{SWEEP_TABLE}}", compare_table(sweep, [r["label"] for r in sweep]))
            .replace("{{TUNE_TABLE}}", compare_table(tune, [r["label"] for r in tune]))
            .replace("{{IMG_C48_3D}}", img(os.path.join(HERE, "cmp_c48_3d.png")))
            .replace("{{IMG_C48_ERR}}", img(os.path.join(HERE, "cmp_c48_errors.png")))
            .replace("{{IMG_C48_TOP}}", img(os.path.join(HERE, "cmp_c48_top.png")))
            .replace("{{IMG_F67_3D}}", img(os.path.join(HERE, "cmp_f67_3d.png")))
            .replace("{{IMG_F67_ERR}}", img(os.path.join(HERE, "cmp_f67_errors.png")))
            .replace("{{IMG_F67_TOP}}", img(os.path.join(HERE, "cmp_f67_top.png"))))
    open(os.path.join(HERE, "report.html"), "w").write(page)
    print("wrote report.html", len(page) // 1024, "kB")


if __name__ == "__main__":
    main()
