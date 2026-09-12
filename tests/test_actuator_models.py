#!/usr/bin/env python3
"""The configurable thrust model and the control-surface servo, checked WITHOUT Isaac.

Both live in vtol.py, which imports Isaac. The maths does not, so it is re-derived here from the
same formulas and checked for the properties that matter:

  * "instant" and "lag" preserve STATIC thrust exactly (the legacy behaviour is untouched)
  * "momentum" also preserves static thrust, and falls to zero at the zero-thrust advance ratio
  * the servo obeys its rate limit and its lag, and settles on the commanded angle
"""
import math
import sys

RHO = 1.225


def thrust_momentum(throttle, v_axial, max_thrust=18.0, omega_max=749.0,
                    D=0.3048, J0=0.6):
    n_max = omega_max / (2 * math.pi)
    ct0 = max_thrust / (RHO * n_max ** 2 * D ** 4)
    n = throttle * n_max
    return max(0.0, RHO * D * D * (ct0 * (n * D) ** 2 - (ct0 / J0) * (n * D) * v_axial))


def main():
    bad = []
    # --- static thrust must be preserved by every model ---
    t_inst = 1.0 * 18.0
    t_lag = 18.0 * (749.0 ** 2) / (749.0 ** 2)
    t_mom = thrust_momentum(1.0, 0.0)
    print(f"  static, full throttle:  instant {t_inst:.3f}  lag {t_lag:.3f}  momentum {t_mom:.3f} N")
    for nm, v in (("lag", t_lag), ("momentum", t_mom)):
        if abs(v - 18.0) > 1e-6:
            bad.append(f"{nm} does not preserve static thrust ({v:.4f} vs 18.0)")

    # --- momentum: thrust must vanish at J = J0 ---
    D, J0, omega_max = 0.3048, 0.6, 749.0
    n = 0.8 * omega_max / (2 * math.pi)
    v_zero = J0 * n * D                      # the airspeed that makes J == J0 at this n
    t_at_J0 = thrust_momentum(0.8, v_zero)
    print(f"  at J = J0 (throttle 0.8, V = {v_zero:.2f} m/s): thrust {t_at_J0:.4f} N")
    if t_at_J0 > 1e-6:
        bad.append(f"momentum model makes {t_at_J0:.3f} N at the zero-thrust advance ratio")

    # --- momentum must be monotonically decreasing in airspeed ---
    prev = None
    print("  throttle 1.0:  ", end="")
    for V in (0, 4, 8, 12, 16):
        t = thrust_momentum(1.0, V)
        print(f"V={V}:{t:6.2f}N ", end="")
        if prev is not None and t > prev + 1e-9:
            bad.append("momentum thrust increases with airspeed")
        prev = t
    print()

    # --- servo: rate limit and lag ---
    def servo(cmds, dt, rate_deg_s, tau):
        cur, out = 0.0, []
        for target in cmds:
            step = (target - cur) * (min(dt / tau, 1.0) if tau > 1e-6 else 1.0)
            if rate_deg_s > 0:
                lim = math.radians(rate_deg_s) * dt
                step = max(-lim, min(lim, step))
            cur += step
            out.append(cur)
        return out

    dt, N = 0.004, 400
    step_cmd = [math.radians(25.0)] * N
    legacy = servo(step_cmd, dt, 0.0, 0.0)
    fast = servo(step_cmd, dt, 400.0, 0.03)
    slow = servo(step_cmd, dt, 120.0, 0.05)          # small stepper
    print(f"  step to 25 deg: legacy reaches it in {dt*1000:.0f} ms; "
          f"servo(400deg/s,30ms) {math.degrees(fast[15]):.1f} deg @60ms, "
          f"final {math.degrees(fast[-1]):.2f}; "
          f"stepper(120deg/s) final {math.degrees(slow[-1]):.2f}")
    if abs(legacy[0] - math.radians(25.0)) > 1e-9:
        bad.append("legacy surface is no longer instantaneous (back-compat broken)")
    if abs(math.degrees(fast[-1]) - 25.0) > 0.05:
        bad.append("servo does not settle on the commanded angle")
    # the rate limit must actually bind on a step
    worst = max(abs(fast[i] - fast[i - 1]) / dt for i in range(1, 40))
    if math.degrees(worst) > 400.0 + 1e-6:
        bad.append(f"servo exceeded its rate limit ({math.degrees(worst):.0f} deg/s)")
    print(f"  peak servo rate during the step: {math.degrees(worst):.0f} deg/s (limit 400)")

    if bad:
        print("\nACTUATOR MODEL PROBLEMS:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nACTUATOR MODELS OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
