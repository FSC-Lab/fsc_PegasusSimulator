#!/usr/bin/env python
"""
Standalone entry point for x650_attitude_sim -- run with a plain Python
interpreter (no Isaac Sim needed) from the repo root:

    python3 -c "
    import sys; sys.path.insert(0, 'extensions/fsc_aerial_manipulation')
    from fsc_aerial_manipulation.x650_attitude_sim.run_validation import main
    main()"

or, equivalently, from extensions/fsc_aerial_manipulation/:

    python3 -m fsc_aerial_manipulation.x650_attitude_sim.run_validation

Produces x650_attitude_validation.png next to this file.
"""
from pathlib import Path

from .simulate import simulate
from .plotting import plot_results

HERE = Path(__file__).resolve().parent


def main():
    log = simulate(t_max=10.0, dt=0.002)
    plot_results(log, 'X650 attitude-only tracking (real rotor calibration/lag + parasitic drag/gyro)',
                 save_path=str(HERE / 'x650_attitude_validation.png'))
    return log


if __name__ == '__main__':
    main()
