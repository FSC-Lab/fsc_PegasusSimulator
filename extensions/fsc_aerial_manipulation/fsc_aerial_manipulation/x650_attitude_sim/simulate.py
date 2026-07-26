#!/usr/bin/env python
"""RK4 closed-loop driver: reference.py + controller.py (AGAS) +
dynamics.py (real X650 rotor calibration/lag/allocation + propeller drag +
rotor gyroscopic torque), attitude-only."""
import numpy as np

from . import dynamics as D
from . import controller as C
from . import reference as R


def simulate(t_max=10.0, dt=0.002, maneuver=None, gains=None, verbose=True):
    gains = GAINS if gains is None else gains
    state = D.initial_state()
    n_steps = int(t_max / dt)

    log = {k: [] for k in ['t', 'Psi_q', 'omega_tilde', 'omega', 'omega_d', 'tau_q', 'rotor_omega']}

    t = 0.0
    for i in range(n_steps):
        ref = R.compute_reference(t, maneuver=maneuver)
        tau_q, info = C.compute_control(state['R'], state['omega'], ref, D.J, D.J_INV, gains)

        log['t'].append(t)
        log['Psi_q'].append(float(info['Psi_q']))
        log['omega_tilde'].append(info['omega_tilde'].copy())
        log['omega'].append(state['omega'].copy())
        log['omega_d'].append(ref['omega_d'].copy())
        log['tau_q'].append(tau_q.copy())
        log['rotor_omega'].append(state['rotor_omega'].copy())

        state = D.rk4_step(state, tau_q, dt)
        t += dt

        if not np.all(np.isfinite(state['omega'])):
            print(f"[simulate] diverged at t={t:.3f}s (step {i}); stopping early.")
            break

        if verbose and i % max(1, n_steps // 10) == 0:
            print(f"[simulate] t={t:6.2f}s  Psi_q={info['Psi_q']:.5f}  "
                  f"|omega_tilde|={np.linalg.norm(info['omega_tilde']):.5f}")

    for k in log:
        log[k] = np.array(log[k])
    return log


# Swept against X650's REAL inertia (diag ~0.058/0.064/0.065 kg*m^2) and
# REAL bench rotor lag (10.51/s) -- NOT just copied from the companion
# Automatica2026Simulation project's placeholder-inertia gains (b_omega=3,
# b_r=10 there; that pair still works here but (4,15) tracks tighter with
# comfortable margin). Sweep (10s doublet maneuver, peak Psi_q / rotor
# saturation as the pass/fail signal): stable and well-tracked through
# (b_omega,b_r)=(8,40); (10,60) and above go unstable (rotor commands
# saturate at the 817.6 rad/s ceiling, peak Psi_q ~2.0 i.e. near-total
# attitude loss). (4,15) sits comfortably inside the stable region with the
# smallest peak tracking error found (peak Psi_q ~4e-5, i.e. <0.5 deg peak
# attitude error) and no rotor saturation (max ~554 rad/s vs the 817.6
# ceiling) -- see CLAUDE.md for the full sweep table.
GAINS = dict(b_omega=4.0, b_r=15.0, delta_R=0.0, epsilon_R=0.05, c2=1.0)
