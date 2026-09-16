#!/usr/bin/env python3
"""Hardware preflight (hardware-transfer piece 5, 2026-09-15): read PX4's parameters back over
MAVLink and REFUSE unless every failsafe the HIL stack relaxes on purpose is armed on the aircraft.

    preflight_hw.py [--url udpin:0.0.0.0:14550] [--table scripts/preflight_hw.cfg] [--timeout 20]
    preflight_hw.py --check '{"COM_LOW_BAT_ACT": 0, ...}'      # offline: judge a parameter map

HIL sets COM_LOW_BAT_ACT 0, COM_DISARM_LAND 0, COM_DISARM_PRFLT 0, GF_ACTION 0, the COM_ARM_EKF_*
thresholds wide open (scripts/restart_hil.sh, start_vtol_smc_hil.sh's 8-relax window) so the sim can
arm through a flaky EKF and fly 20 m/s to 800 m. Hardware inverts every one of them, and a launcher
that reports READY without reading them back is the sim-battery Land failsafe of 2026-09-07 waiting
to happen in the other direction. The table (preflight_hw.cfg) is the flight lead's: one line per
parameter, `NAME  op  value  # why` with op in = != < <= > >= in; a parameter PX4 does not have
(SIM_* on hardware) is reported and skipped only if the line says `optional`.
Exit 0 = every line holds; 1 = a refusal (each one printed); 2 = no link / no params."""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def load_table(path):
    rows = []
    for ln in open(path):
        ln = ln.split("#", 1)[0].strip()
        if not ln:
            continue
        parts = ln.split()
        name, op, val = parts[0], parts[1], parts[2]
        optional = "optional" in parts[3:]
        vals = [float(v) for v in val.split(",")]
        rows.append((name, op, vals, optional))
    return rows


def judge(params, table):
    """params: {name: value}; returns (ok, lines)."""
    ok = True; out = []
    for name, op, vals, optional in table:
        if name not in params:
            if optional:
                out.append(f"  {name:20s} -- not on this vehicle (optional)"); continue
            out.append(f"  {name:20s} MISSING -- the vehicle has no such parameter"); ok = False; continue
        v = float(params[name]); want = vals[0]
        holds = {"=": v == want, "!=": v != want, "<": v < want, "<=": v <= want, ">": v > want, ">=": v >= want,
                 "in": v in vals}[op]
        out.append(f"  {name:20s} {v:>10g}   {'ok ' if holds else 'REFUSED'}  (want {op} {','.join(f'{w:g}' for w in vals)})")
        ok = ok and holds
    return ok, out


def read_params(url, names, timeout):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(url, source_system=250)
    hb = m.wait_heartbeat(timeout=timeout)
    if hb is None:
        return None
    got = {}
    for name in names:
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 3.0:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg is None:
                continue
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="replace")
            pid = pid.rstrip("\x00")
            if pid == name:
                got[name] = float(msg.param_value); break
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="udpin:0.0.0.0:14550")
    ap.add_argument("--table", default=os.path.join(HERE, "preflight_hw.cfg"))
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--check", default=None, help="a JSON parameter map to judge offline (no MAVLink)")
    a = ap.parse_args()
    table = load_table(a.table)
    if a.check is not None:
        params = json.loads(a.check)
    else:
        params = read_params(a.url, [r[0] for r in table], a.timeout)
        if params is None:
            print(f"preflight_hw: no MAVLink heartbeat on {a.url} within {a.timeout:.0f} s"); return 2
        if not params:
            print("preflight_hw: heartbeat but no PARAM_VALUE answers"); return 2
    ok, lines = judge(params, table)
    print(f"preflight_hw: {len(table)} parameters against {a.table}")
    print("\n".join(lines))
    print("PREFLIGHT OK -- every failsafe armed" if ok else "PREFLIGHT REFUSED -- not READY")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
