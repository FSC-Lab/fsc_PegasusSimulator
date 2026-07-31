#!/usr/bin/env python
"""Plotting for x650_attitude_sim logs."""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_results(log, title, save_path=None):
    t = log['t']
    fig = plt.figure(figsize=(12, 9))
    fig.suptitle(title)

    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(t, log['Psi_q'])
    ax1.set_xlabel('t [s]'); ax1.set_ylabel('Psi_q')
    ax1.set_title('Attitude tracking error Psi_q (0 = perfect)')
    ax1.grid(True)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(t, np.linalg.norm(log['omega_tilde'], axis=1))
    ax2.set_xlabel('t [s]'); ax2.set_ylabel('|omega_tilde|')
    ax2.set_title('Body-rate tracking error norm')
    ax2.grid(True)

    ax3 = fig.add_subplot(2, 2, 3)
    labels = ['x (roll)', 'y (pitch)', 'z (yaw)']
    for k in range(3):
        ax3.plot(t, log['omega_d'][:, k], '--', color=f'C{k}', label=f'omega_d,{labels[k]}')
        ax3.plot(t, log['omega'][:, k], color=f'C{k}', label=f'omega,{labels[k]}')
    ax3.set_xlabel('t [s]'); ax3.set_ylabel('omega [rad/s]')
    ax3.set_title('Body rate: desired vs actual')
    ax3.legend(fontsize=7); ax3.grid(True)

    ax4 = fig.add_subplot(2, 2, 4)
    for j in range(log['rotor_omega'].shape[1]):
        ax4.plot(t, log['rotor_omega'][:, j], label=f'rotor {j}')
    ax4.set_xlabel('t [s]'); ax4.set_ylabel('rotor omega [rad/s]')
    ax4.set_title('Actual (lagged) rotor speeds')
    ax4.legend(fontsize=7); ax4.grid(True)

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=130)
        print(f"[plot] saved {save_path}")
    plt.close(fig)
