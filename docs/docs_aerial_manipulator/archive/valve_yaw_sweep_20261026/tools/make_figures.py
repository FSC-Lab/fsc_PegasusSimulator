#!/usr/bin/env python3
"""Figures + the run table of the heading-channel sweep (2026-10-26), from ../analysis/scores.json
(tools/yaw_score.py) and the runs:

    PYTHONNOUSERSITE=1 /usr/bin/python3 make_figures.py [--cells yw_k1_f150 ...]

  ../figures/map.png              tau_f x K_psi grid, every run in its cell: outcome, M_bar, peak tilt
  ../figures/split.png            per run: the couple M, F_t r and their sum about the axis vs tau_f
  ../figures/cell_<tag>.png       one chosen run: the couple measured vs rendered over the contact
                                  phase; tilt, the commanded yaw torque, q1 / q4 over the turn
  ../analysis/run_table.md        one row per run, every check
"""
import argparse
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SWEEP = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import yaw_score as YS   # noqa: E402

FIG = os.path.join(SWEEP, "figures")
GREEN, AMBER, RED, GREY = "#2e8b57", "#e0a020", "#c0392b", "#bbbbbb"


def outcome_color(r):
    if not r.get("complete"):
        return RED
    return GREEN if r.get("safe") else AMBER


def kgroup(r):
    k = r["K_psi"]
    return "0.25" if abs(k - 0.2484) < 1e-3 else f"{k:g}"


def fgroup(r):
    f = r["tau_f"]
    return "0.05" if f < 0.06 else f"{f:.2f}"


def run_label(tag):
    return tag.replace("yw_", "")


def short_mode(m):
    s = "tilt guard" if "tilt" in m else ("q3 guard" if "q3" in m else ("force guard" if "force" in m else m[:12]))
    if "SAFETY" in m:
        s += "\n+SAFETY"
    if "inverted" in m:
        s += "\nINVERTED"
    return s


def make_map(db, anchor_off=False):
    rows = [r for r in db.values() if ("fa0" in r["run"]) == anchor_off]
    Ks = sorted({kgroup(r) for r in rows}, key=float)
    Fs = sorted({fgroup(r) for r in rows}, key=float)
    fig, ax = plt.subplots(figsize=(max(8.0, 2.1 * len(Fs) + 1.6), 1.05 * len(Ks) + 1.6))
    for i, K in enumerate(Ks):
        for j, F in enumerate(Fs):
            cell = sorted([r for r in rows if kgroup(r) == K and fgroup(r) == F], key=lambda r: r["run"])
            if not cell:
                ax.add_patch(plt.Rectangle((j, i), 1, 1, color="#f4f4f4", ec="white"))
                continue
            n = len(cell)
            for k, r in enumerate(cell):
                ax.add_patch(plt.Rectangle((j + k / n, i), 1 / n, 1, color=outcome_color(r), ec="white", lw=1.5))
                txt = (f"{r['M_bar']:+.3f}\n{r.get('tilt_turn_max', np.nan):.1f}°"
                       if r.get("complete") else f"{short_mode(r.get('mode', ''))}\n{r.get('tilt_turn_max', np.nan):.0f}°")
                ax.text(j + (k + 0.5) / n, i + 0.5, txt, ha="center", va="center", fontsize=7.5, color="white")
    ax.set_xlim(0, len(Fs)); ax.set_ylim(0, len(Ks))
    ax.set_xticks(np.arange(len(Fs)) + 0.5); ax.set_xticklabels([f"τ_f {f}" for f in Fs])
    ax.set_yticks(np.arange(len(Ks)) + 0.5); ax.set_yticklabels([f"K_ψ {k}" for k in Ks])
    ax.set_title(("FREE-AXIS ANCHOR OFF (diagnostic)" if anchor_off else "Shipped valve config (free-axis anchor 0.75)")
                 + " -- 90° in 12 s, stick-slip bearing\ngreen = every safety check, amber = complete but a check failed, "
                 "red = guard / SAFETY handover.\nCell text: mean couple M̄ [N·m] (resisting +), peak tilt in the turn",
                 fontsize=9)
    fig.tight_layout()
    p = os.path.join(FIG, "map_anchor_off.png" if anchor_off else "map.png"); fig.savefig(p, dpi=130); plt.close(fig); return p


def make_split(db):
    rows = sorted([r for r in db.values() if r.get("complete")], key=lambda r: (r["K_psi"], r["tau_f"], r["run"]))
    fig, ax = plt.subplots(figsize=(max(7, 0.7 * len(rows) + 2), 4.2))
    x = np.arange(len(rows))
    M = np.array([r["M_bar"] for r in rows]); Ftr = np.array([r["Ftr_bar"] for r in rows])
    ax.bar(x - 0.2, M, 0.4, label="couple M̄ (heading channel)", color="#3465a4")
    ax.bar(x + 0.2, Ftr, 0.4, label="F̄_t · r (translational channel)", color="#999999")
    ax.plot(x, M + Ftr, "k_", ms=18, mew=2, label="M̄ + F̄_t r (the valve's load)")
    ax.plot(x, [r["tau_f"] for r in rows], "r_", ms=18, mew=1.2, label="τ_f set in the scene")
    ax.plot(x, [r["M_pred"] for r in rows], "x", color="#3465a4", label="M predicted, K_ψ τ_f / (K r² + K_ψ)")
    ax.axhline(0, color="k", lw=0.5)
    ax.axhline(YS.M_VIS, color="#3465a4", lw=0.6, ls=":")
    ax.set_xticks(x); ax.set_xticklabels([run_label(r["run"]) for r in rows], rotation=60, ha="right", fontsize=7.5)
    ax.set_ylabel("moment about the axis, resisting + [N·m]\n(middle 60 % of the turn)")
    ax.legend(fontsize=7.5, ncol=2); ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    p = os.path.join(FIG, "split.png"); fig.savefig(p, dpi=130); plt.close(fig); return p


def make_cell(tag):
    row, S = YS.score(tag)
    t0 = S["a"]
    fig, ax = plt.subplots(4, 1, figsize=(9, 9.5), sharex=True)
    ax[0].plot(S["t"] - t0, S["M"], label="measured couple (contact truth)")
    ax[0].plot(S["t"] - t0, S["R"], ":", label="rendered M_ψ ë + D_ψ ė + K_ψ e")
    ax[0].plot(S["t"] - t0, S["Ft"] * S["r"], color="0.6", lw=0.8, label="F_t r")
    ax[0].axhline(row["tau_f"], color="r", lw=0.6, label="τ_f")
    ax[0].set_ylabel("resisting + [N·m]")
    m = (S["bt_t"] > S["tA"] - 2) & (S["bt_t"] < S["tE"] + 4)
    ax[1].plot(S["bt_t"][m] - t0, S["tilt"][m]); ax[1].axhline(7, color="r", lw=0.6); ax[1].set_ylabel("tilt [deg]")
    m = (S["dbg_t"] > S["tA"] - 2) & (S["dbg_t"] < S["tE"] + 4)
    ax[2].plot(S["dbg_t"][m] - t0, S["tz"][m], lw=0.6)
    ax[2].axhline(0.7 * row["yaw_ceiling"], color="r", lw=0.6, label="70 % of the allocator's yaw ceiling")
    ax[2].set_ylabel("yaw torque cmd [N·m]")
    m = (S["js_t"] > S["tA"] - 2) & (S["js_t"] < S["tE"] + 4)
    ax[3].plot(S["js_t"][m] - t0, S["q"][m, 0], label="q1"); ax[3].plot(S["js_t"][m] - t0, S["q"][m, 3], label="q4")
    ax[3].set_ylabel("[deg]"); ax[3].set_xlabel("s from the turn's start")
    for x in ax:
        x.axvspan(0, S["b"] - S["a"], color="0.92"); x.axvspan(S["mid"][0] - t0, S["mid"][1] - t0, color="0.85")
        x.grid(alpha=0.3); x.legend(fontsize=7)
    fig.suptitle(f"{tag}: K_ψ {row['K_psi']:g}, D_ψ {row['D_psi']:.3g}, τ_f {row['tau_f']:.3f} N·m", fontsize=10)
    fig.tight_layout()
    p = os.path.join(FIG, f"cell_{tag}.png"); fig.savefig(p, dpi=110); plt.close(fig); return p


def run_table(db):
    cols = [("run", "{}"), ("K_psi", "{:.3g}"), ("D_psi", "{:.3g}"), ("tau_f", "{:.3f}"), ("outcome", "{}"),
            ("tilt_turn_max", "{:.1f}"), ("yaw_torque_frac", "{:.0%}"), ("motor_sat", "{}"), ("head_err_max_deg", "{:.1f}"),
            ("q1_std", "{:.1f}"), ("q4_std", "{:.1f}"), ("q1_growth", "{:.2f}"), ("q1_growth_mid", "{:.2f}"),
            ("q1_peak_hz", "{:.2f}"), ("on_stop_frac", "{:.0%}"), ("q3_min", "{:.1f}"), ("task_force_peak", "{:.2f}"),
            ("eps_turn_max_mm", "{:.0f}"), ("turned_deg", "{:.1f}"), ("back_after_release_deg", "{:.1f}"),
            ("M_bar", "{:+.3f}"), ("M_sigma", "{:.3f}"), ("R_bar", "{:+.3f}"), ("corr_turn", "{:+.2f}"),
            ("Ftr_bar", "{:+.3f}"), ("axis_sum_mid", "{:+.3f}"), ("share_meas", "{:+.2f}"), ("share_pred", "{:.2f}"),
            ("lag_mid_deg", "{:+.1f}"), ("lag_pred_deg", "{:.2f}"), ("failed", "{}")]
    out = ["| " + " | ".join(c for c, _ in cols) + " |", "|" + "---|" * len(cols)]
    for r in sorted(db.values(), key=lambda r: (("fa0" in r["run"]), r["K_psi"], r["tau_f"], r["run"])):
        r = dict(r)
        r["outcome"] = "complete" if r.get("complete") else ("FAIL " + r.get("mode", ""))
        cells = []
        for c, f in cols:
            v = r.get(c, "")
            try:
                cells.append(f.format(v) if v != "" else "")
            except (ValueError, TypeError):
                cells.append(str(v))
        out.append("| " + " | ".join(cells) + " |")
    p = os.path.join(SWEEP, "analysis", "run_table.md")
    open(p, "w").write("\n".join(out) + "\n")
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cells", nargs="*", default=[])
    a = ap.parse_args()
    os.makedirs(FIG, exist_ok=True)
    db = json.load(open(os.path.join(SWEEP, "analysis", "scores.json")))
    for p in [make_map(db), make_map(db, anchor_off=True), make_split(db), run_table(db)] + [make_cell(t) for t in a.cells]:
        print("wrote", p)


if __name__ == "__main__":
    main()
