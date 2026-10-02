#!/usr/bin/env python3
"""Bench gain names -> the whole-body yaml keys (one mapping for the Isaac
overrides AND the final yaml update, so the two cannot drift).

    gains_to_yaml.py <json with "best"> --replay        # WB_REPLAY_GAINS string
    gains_to_yaml.py <json with "best"> --apply f.yaml [f2.yaml ...] --note "..."
"""
import argparse
import json
import re
import sys

MRD0 = (0.130710, 0.135962, 0.134261)


def keys(p):
    k = {"wb_k_x": p["k_x"], "wb_k_v": p["k_v"], "wb_k_r": p["k_R"], "wb_k_w": p["k_w"],
         "wb_mrd_x": MRD0[0] * p["mrd_s"], "wb_mrd_y": MRD0[1] * p["mrd_s"], "wb_mrd_z": MRD0[2] * p["mrd_s"],
         "wb_ky_x": p["ky"], "wb_ky_y": p["ky"], "wb_ky_z": p["ky"], "wb_ky_psi": p["ky_psi"],
         "wb_dy_x": p["dy"], "wb_dy_y": p["dy"], "wb_dy_z": p["dy"], "wb_dy_psi": p["dy_psi"],
         "wb_l1_omega_c_t": p["omega_c_t"], "wb_l1_omega_c_r": p["omega_c_r"],
         "wb_l1_omega_c_q": p["omega_c_q"], "wb_l1_omega_x": p["omega_x"]}
    return {n: float(f"{v:.4g}") if not n.startswith("wb_mrd") else round(v, 6) for n, v in k.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json"); ap.add_argument("--replay", action="store_true")
    ap.add_argument("--apply", nargs="*", default=[]); ap.add_argument("--note", default="")
    a = ap.parse_args()
    p = json.load(open(a.json))["best"]
    K = keys(p)
    if a.replay:
        print(",".join(f"{n}={v}" for n, v in K.items()))
    for f in a.apply:
        s = open(f).read()
        for n, v in K.items():
            m = re.search(rf"^(\s*{n}:[ \t]*)([-0-9.eE+]+)(.*)$", s, re.M)
            if not m:
                sys.exit(f"{f}: key {n} not found")
            old = m.group(2); txt = f"{v:g}"
            if re.fullmatch(r"-?\d+", txt):
                txt += ".0"
            tail = m.group(3)
            if abs(float(old) - float(txt)) <= 1e-9 * max(1.0, abs(float(old))):
                txt = old                     # numerically unchanged: keep the file's text
            else:
                tail = f"  # 2026-09-27 circle tune; was {old}" + (f" ({a.note})" if a.note else "")
            s = s[:m.start()] + m.group(1) + txt + tail + s[m.end():]
        open(f, "w").write(s)
        print(f"applied {len(K)} keys to {f}")


if __name__ == "__main__":
    main()
