"""
utils_plot — figure generation from a run's .npz log.

    plot_results.py   the MATLAB-style figure set (states, tracking errors,
                      control effort, 2-D trajectories, 3-D trajectory), with
                      flight-phase shading driven by the npz's phase_t/phase_names

Run it directly; it needs no arguments, defaulting to the newest log:

    ~/isaacsim/python_r_fsc.sh <this dir>/plot_results.py --tag showcase

Paths are anchored to the robotic_arm PACKAGE, not to this folder — a run's
output belongs to the scenario, not to the code that renders it:

    robotic_arm/results/log/       .npz written by the demos   (read)
    robotic_arm/results/figures/   .png written by this script (written)
"""
