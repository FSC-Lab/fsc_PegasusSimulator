# Aerial Manipulation

Whole-body controller for a quadrotor (x650) + OpenManipulator-X 4-DoF arm in
Isaac Sim / Pegasus. Two demo variants, sharing the same `AM_realign.usda` model.

## Run

```bash
# Run from the repo root. <config> = a file in scripts/config/ (e.g. shiqi_machine).

# Hover — distributed ROS2 (Isaac + controller node, two tmux panes)
~/fsc_PegasusSimulator$ ./scripts/start_aerial_manipulator_hover.sh <config>

# Track — in-process, no ROS2 (single Isaac process, 250 Hz control)
~/fsc_PegasusSimulator$ ./scripts/start_aerial_manipulator_track.sh <config>
```

## Scripts

| Role       | Hover (ROS2)                                          | Track (in-process)                                    |
|------------|-------------------------------------------------------|-------------------------------------------------------|
| Isaac demo | `application/robotic_arm/01_aerial_manipulator_hover.py` | `application/robotic_arm/01_aerial_manipulator_track.py` |
| Controller | `controller_hover.py` (this folder)                   | `controller_track.py` (this folder)                   |
| Launcher   | `scripts/start_aerial_manipulator_hover.sh`           | `scripts/start_aerial_manipulator_track.sh`           |

Supporting modules, one package per concern:
`utils_planner/` (desired trajectories), `utils_controller/` (model + control
law), `utils_vehicle/` (`x650_vehicle.py` / `x650_multirotor.py`),
`utils_model/` (`postprocessor.py`, one-shot USD authoring),
`utils_plot/` (`plot_results.py`).

## Plot a run

Use the Isaac python (`ISAAC_PY` in `scripts/config/`) — the system `python3`'s
matplotlib may not match its numpy.

```bash
# reads log/track_log.npz, writes PNGs to images/
~/fsc_PegasusSimulator$ ~/isaacsim/python_r_fsc.sh extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_plot/plot_results.py
```

## Outputs

- Data log — `log/track_log.npz` (track demo only): written automatically on clean exit; **close the Isaac Sim window** to trigger it (Ctrl+C in the terminal may skip it)
- Plots — `results/figures/*.png`, from `utils_plot/plot_results.py`
