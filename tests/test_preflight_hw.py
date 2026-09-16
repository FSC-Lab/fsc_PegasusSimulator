#!/usr/bin/env python3
"""The hardware preflight (piece 5, 2026-09-15): scripts/preflight_hw.py against its own table.
  1. a parameter map with every failsafe armed passes;
  2. each HIL relaxation, ONE at a time (the sim battery Land off, disarm-on-land off, the EKF gates
     wide, the geofence off, offboard loss to a manual mode, no kill switch, the pitot undeclared),
     is refused and the refusal names the parameter;
  3. a sim-only parameter present (SIM_BAT_MIN_PCT) is refused; absent it is skipped;
  4. a required parameter the vehicle does not have is refused;
  5. (live, if a PX4 heartbeat is on 14550) the HIL stack itself is REFUSED -- its parameters are the sim's.
    /usr/bin/python3 tests/test_preflight_hw.py
"""
import importlib.util
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
spec = importlib.util.spec_from_file_location("preflight_hw", os.path.join(ROOT, "scripts", "preflight_hw.py"))
pf = importlib.util.module_from_spec(spec); spec.loader.exec_module(pf)
fails = 0


def check(cond, msg):
    global fails
    print(("PASS " if cond else "FAIL ") + msg); fails += 0 if cond else 1


ARMED = {"COM_LOW_BAT_ACT": 3, "BAT_LOW_THR": 0.15, "COM_DISARM_LAND": 2.0, "COM_DISARM_PRFLT": 10.0,
         "COM_ARM_EKF_HGT": 1.0, "COM_ARM_EKF_VEL": 0.5, "COM_ARM_EKF_POS": 0.5, "COM_ARM_EKF_YAW": 0.5,
         "COM_ARM_IMU_ACC": 0.7, "COM_ARM_IMU_GYR": 0.25, "COM_ARM_MAG_ANG": 45,
         "GF_ACTION": 2, "GF_MAX_HOR_DIST": 600, "GF_MAX_VER_DIST": 120, "GF_PREDICT": 1,
         "COM_OF_LOSS_T": 1.0, "COM_OBL_ACT": 4, "NAV_RCL_ACT": 2, "RC_MAP_KILL_SW": 7,
         "SYS_HAS_NUM_ASPD": 1, "CBRK_SUPPLY_CHK": 0, "CBRK_USB_CHK": 0, "CBRK_AIRSPD_CHK": 0}
# what the HIL stack sets, one at a time
HIL = {"COM_LOW_BAT_ACT": 0, "BAT_LOW_THR": 0.05, "COM_DISARM_LAND": 0, "COM_DISARM_PRFLT": 0,
       "COM_ARM_EKF_HGT": 3.0, "COM_ARM_EKF_VEL": 1.0, "COM_ARM_EKF_POS": 1.0, "COM_ARM_EKF_YAW": 1.0,
       "COM_ARM_IMU_ACC": 1.0, "COM_ARM_IMU_GYR": 1.0, "COM_ARM_MAG_ANG": -1,
       "GF_ACTION": 0, "GF_MAX_HOR_DIST": 0, "GF_MAX_VER_DIST": 0, "GF_PREDICT": 0,
       "COM_OF_LOSS_T": 5.0, "COM_OBL_ACT": 0, "NAV_RCL_ACT": 0, "RC_MAP_KILL_SW": 0,
       "SYS_HAS_NUM_ASPD": 0, "CBRK_SUPPLY_CHK": 894281, "CBRK_USB_CHK": 197848, "CBRK_AIRSPD_CHK": 162128}


def main():
    table = pf.load_table(os.path.join(ROOT, "scripts", "preflight_hw.cfg"))
    ok, lines = pf.judge(ARMED, table)
    check(ok, f"every failsafe armed: passes ({len(table)} lines)")
    for name, bad in HIL.items():
        m = dict(ARMED); m[name] = bad
        ok, lines = pf.judge(m, table)
        named = any(name in ln and "REFUSED" in ln for ln in lines)
        check((not ok) and named, f"HIL's {name}={bad} alone is refused and named")
    m = dict(ARMED); m["SIM_BAT_MIN_PCT"] = 100
    ok, lines = pf.judge(m, table)
    check(not ok and any("SIM_BAT_MIN_PCT" in ln and "REFUSED" in ln for ln in lines), "a sim battery parameter PRESENT is refused (this is a simulator)")
    ok, lines = pf.judge(ARMED, table)
    check(ok and any("SIM_BAT_MIN_PCT" in ln and "optional" in ln for ln in lines), "absent, the optional line is skipped")
    m = dict(ARMED); del m["RC_MAP_KILL_SW"]
    ok, lines = pf.judge(m, table)
    check(not ok and any("RC_MAP_KILL_SW" in ln and "MISSING" in ln for ln in lines), "a required parameter the vehicle lacks is refused")
    # the CLI, offline
    r = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "preflight_hw.py"), "--check", "{}"], capture_output=True, text=True)
    check(r.returncode == 1 and "PREFLIGHT REFUSED" in r.stdout, "the CLI refuses an empty map (exit 1)")
    # live: the HIL stack, if any, must be refused
    try:
        params = pf.read_params("udpin:0.0.0.0:14550", [row[0] for row in table], 5.0)
    except Exception as e:                       # noqa: BLE001
        params = None; print(f"  (live check skipped: {e})")
    if params:
        ok, lines = pf.judge(params, table)
        print("\n".join(lines))
        check(not ok, "the LIVE HIL stack is refused (its parameters are the simulator's)")
    else:
        print("  (no PX4 heartbeat on 14550: live check skipped)")
    print()
    print("ALL PASS" if fails == 0 else f"{fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
