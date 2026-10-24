#!/usr/bin/env python3
"""Markdown tables from ../stats.json (hover_stats.py). Usage: summary.py [estimate C|M] > tables.md"""
import json, sys, os
import numpy as np
OUT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
S = json.load(open(f"{OUT}/stats.json"))
EST = sys.argv[1] if len(sys.argv) > 1 else "C"
f1 = lambda x: "–" if x is None else f"{x:.1f}"
f0 = lambda x: "–" if x is None else f"{x:.0f}"


def comp_row(name, s):
    out = []
    for k in ("along", "lateral", "vertical"):
        c = s[k]
        out.append(f"{f1(c['mean'])} / {f1(c['std'])} / {f0(c['p95_abs'])} / {f0(c['p99_abs'])} / {f0(c['max_abs'])}")
    h, tl, n3 = s["heading_deg"], s["tilt_deg"], s["norm3d"]
    return (f"| {name} | {s['n_windows']} / {s['seconds']:.0f} s | " + " | ".join(out)
            + f" | {f0(n3['rms'])} / {f0(n3['p95'])} / {f0(n3['max_abs'])}"
            + f" | {h['mean']:.2f} / {h['std']:.2f} / {h['p95_abs']:.1f} | {tl['mean']:.1f} / {tl['p95']:.1f} / {tl['max_abs']:.1f} |")


HDR = ("| pool | windows / time | along-nose mean / std / p95 / p99 / max [mm] | lateral (same) | vertical (same) "
       "| 3-D rms / p95 / max [mm] | EE heading mean / std / p95 [deg] | tilt mean / p95 / max [deg] |\n|" + "---|" * 8)


def table(fset, kind, groups, loads=("unloaded", "loaded")):
    print(HDR)
    for g in groups:
        for load in loads:
            s = S["pooled"].get(fset, {}).get(EST, {}).get(kind, {}).get(g, {}).get(load)
            if s:
                print(comp_row(f"{g}, {load}", s))
    print()


for fset in ("main", "supp", "all"):
    for kind, label in (("full", "from the leg end"), ("s1", "from 1 s after the leg end"), ("settled", "from 3 s after the leg end (spec)")):
        print(f"### set {fset} — {label} — estimate {EST}\n")
        table(fset, kind, ("gripper_holds", "approach_wait", "at_pick", "at_drop", "above_handle", "at_handle", "other", "all_free",
                           "direct_entry", "post_abort", "post_lift", "bin"))

print("### window decomposition (set all, estimate %s): standing offset = rms of the per-window means, wander = pooled within-window std\n" % EST)
print("| pool | settle | along offset / wander | lateral offset / wander | vertical offset / wander | window means min..max along / lat / vert |\n|---|---|---|---|---|---|")
for kind in ("full", "s1", "settled"):
    for g in ("gripper_holds", "other", "all_free"):
        for load in ("unloaded", "loaded"):
            s = S["pooled"].get("all", {}).get(EST, {}).get(kind, {}).get(g, {}).get(load)
            if not s:
                continue
            cells = [f"{s[k]['window_mean_rms']:.1f} / {s[k]['within_window_std']:.1f}" for k in ("along", "lateral", "vertical")]
            rng = " / ".join(f"{s[k]['window_mean_min']:.0f}..{s[k]['window_mean_max']:.0f}" for k in ("along", "lateral", "vertical"))
            print(f"| {g}, {load} | {kind} | " + " | ".join(cells) + f" | {rng} |")
print()

print("### wander frequency (windows >= 6 s)\n")
for fset in ("main", "supp", "all"):
    for kind in ("full", "settled"):
        for g in ("all_free",):
            for load in ("unloaded", "loaded"):
                s = S["pooled"].get(fset, {}).get(EST, {}).get(kind, {}).get(g, {}).get(load)
                if not s or not s["frequency"]["n_windows"]:
                    continue
                fr = s["frequency"]
                print(f"- set {fset}, {kind}, {g}, {load}: {fr['n_windows']} windows, {fr['seconds']:.0f} s")
                for k in ("along", "lateral", "vertical"):
                    c = fr[k]
                    print(f"  - {k}: mean-PSD peak {c['peak_of_mean_psd_hz']:.2f} Hz, window peaks {c['window_peaks_hz']}, "
                          f"zero-crossing {c['median_zero_crossing_hz']:.2f} Hz, band fractions "
                          + ", ".join(f"{b} {v:.2f}" for b, v in c["band_fraction"].items()))
print()

print("### arrival (3-D claw error, estimate %s): time from the leg end until inside 10 / 20 / 30 mm\n" % EST)
print("| set, load, group | legs | median window [s] | |e| at leg end median [mm] | 1 s dwell inside 10 / 20 / 30 mm: median (reached of legs) | stays inside to window end: median (reached) |\n|---|---|---|---|---|---|")
for fset in ("main", "supp", "all"):
    for load in ("unloaded", "loaded"):
        for g in ("gripper_holds", "other", "all_free"):
            a = S["arrival_summary"].get(fset, {}).get(EST, {}).get(load, {}).get(g)
            if not a:
                continue
            dw = " / ".join(f"{f1(a[f'dwell1s_{t}']['median'])} s ({a[f'dwell1s_{t}']['reached']})" for t in (10, 20, 30))
            st = " / ".join(f"{f1(a[f'stay_{t}']['median'])} s ({a[f'stay_{t}']['reached']})" for t in (10, 20, 30))
            print(f"| {fset}, {load}, {g} | {a['n']} | {a['window_s_median']:.1f} | {a['e_at_leg_end_median_mm']:.0f} | {dw} | {st} |")
print()

print("### arrival curve (estimate %s): 3-D claw error at fixed times after the leg end, over the legs still holding then\n" % EST)
print("| set, load, group | t after leg end [s] | legs still holding | median / p90 / max [mm] | share <= 10 / 20 / 30 mm |\n|---|---|---|---|---|")
for fset in ("main", "all"):
    for load in ("unloaded", "loaded"):
        for g in ("gripper_holds", "all_free"):
            a = S["arrival_summary"].get(fset, {}).get(EST, {}).get(load, {}).get(g)
            if not a:
                continue
            for c, v in a["error_vs_time_after_leg_end"].items():
                print(f"| {fset}, {load}, {g} | {c} | {v['n']} | {v['median']:.0f} / {v['p90']:.0f} / {v['max']:.0f} | "
                      f"{100 * v['frac_le_10']:.0f} / {100 * v['frac_le_20']:.0f} / {100 * v['frac_le_30']:.0f} % |")
print()

print("### error at the gripper command (mean over 0.25..0.05 s before the jaws move), estimate %s\n" % EST)
print("| flight | event | load | hold before [s] | along / lateral / vertical [mm] | 3-D [mm] | heading [deg] | tilt [deg] |\n|---|---|---|---|---|---|---|---|")
for e in S["events"]:
    if e["est"] != EST:
        continue
    print(f"| {e['flight']} | {e['event']} | {e['load']} | {e['hold_s']:.2f} | {e['along']:.0f} / {e['lateral']:.0f} / {e['vertical']:.0f} | {e['norm3d']:.0f} | {e['head_deg']:.2f} | {e['tilt_deg']:.1f} |")
print()
ev = [e for e in S["events"] if e["est"] == EST and e["event"] in ("close_at_pick", "open_at_drop", "close_at_handle")]
for load in ("unloaded", "loaded"):
    E = np.array([[e["along"], e["lateral"], e["vertical"], e["norm3d"]] for e in ev if e["load"] == load])
    if len(E):
        print(f"- events {load} (n={len(E)}): mean along/lat/vert {E[:, 0].mean():.1f} / {E[:, 1].mean():.1f} / {E[:, 2].mean():.1f} mm, "
              f"max |.| {np.abs(E[:, 0]).max():.0f} / {np.abs(E[:, 1]).max():.0f} / {np.abs(E[:, 2]).max():.0f} mm, 3-D median {np.median(E[:, 3]):.0f} max {E[:, 3].max():.0f} mm")
print()

print("### cross-checks\n")
for fid, ck in S["cross_checks"].items():
    a = ck.get("arrival_vs_C"); m = ck["M_minus_C_in_holds"]
    print(f"- {S['meta']['flights'][fid]['label']}: planner arrival_error vs C "
          + (f"n={a['n']}, rms {a['rms_mm']} mm, max {a['max_mm']:.1f} mm" if a else "n/a")
          + f"; M - C in holds mean {m['mean_mm']} std {m['std_mm']} mm, tilt M {m['tilt_M_deg']:.1f} vs PX4 {m['tilt_C_deg']:.1f} deg")

print("\n### per flight (gripper holds + other hovers >= 1.5 s), estimate %s: mean / std / p95|.| [mm]\n" % EST)
print("| flight | set | settle | load | windows / time | along | lateral | vertical | 3-D p95 / max | heading std [deg] |\n|---|---|---|---|---|---|---|---|---|---|")
for fid, per in S["per_flight"][EST].items():
    lab = S["meta"]["flights"][fid]["label"]; st = S["meta"]["flights"][fid]["set"]
    for kind in ("full", "settled"):
        for load in ("unloaded", "loaded"):
            s = per.get(kind, {}).get(load)
            if not s:
                continue
            c = " | ".join(f"{s[k]['mean']:.0f} / {s[k]['std']:.0f} / {s[k]['p95_abs']:.0f}" for k in ("along", "lateral", "vertical"))
            print(f"| {lab} | {st} | {kind} | {load} | {s['n_windows']} / {s['seconds']:.1f} s | {c} | {s['norm3d']['p95']:.0f} / {s['norm3d']['max_abs']:.0f} | {s['heading_deg']['std']:.2f} |")
