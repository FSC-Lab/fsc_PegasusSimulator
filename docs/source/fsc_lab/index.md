# FSC Lab Launch Scripts

The FSC Lab fork ships a set of launch scripts under `scripts/` at the repository root. Each one starts PX4 SITL and Isaac Sim/Pegasus together inside a `tmux` session (opened in a new terminal window by default) so a full scenario comes up with a single command.

## Command pattern

```bash
./scripts/<script_name>.sh <machine_config_name>
```

For example, to launch the single-drone slung-load payload SITL simulation on Longhao's machine:

```bash
./scripts/start_single_drone_sitl_payload.sh longhao_machine
```

`longhao_machine` is not a placeholder — it is the name (without `.conf`) of a file under `scripts/config/`. Every script takes exactly one argument: your machine config name.

## Machine configs — every user adds their own

Paths differ per machine (where PX4 is checked out, where this repo lives, where the Isaac Sim python launcher is) so they are kept out of the scripts and out of shared logic, in `scripts/config/`. **If you don't see a config for your machine, add one** — do not reuse someone else's:

```bash
# scripts/config/<your_name>_machine.conf
PX4_DIR="$HOME/PX4-Autopilot"
FSC_PEGASUS_ROOT="/absolute/path/to/fsc_PegasusSimulator"
ISAAC_PY="$HOME/isaacsim/python_r_fsc.sh"
```

| Variable | Meaning |
|---|---|
| `PX4_DIR` | Root of your local `PX4-Autopilot` checkout (must already be built: `make px4_sitl_default`) |
| `FSC_PEGASUS_ROOT` | Absolute path to the root of this repository on your machine |
| `ISAAC_PY` | Path to the Isaac Sim python launch wrapper (`python_r_fsc.sh`) set up during Isaac Sim installation |

All three variables are required — `scripts/common_config.sh` validates they're set and that the paths exist before launching anything. Once the file exists, launch with its basename (without `.conf`); e.g. `scripts/config/smith_machine.conf` is invoked as:

```bash
./scripts/start_single_drone_sitl.sh smith_machine
```

Configs already checked into the repo (for reference only — add your own rather than reusing one of these):

| Config name | Machine |
|---|---|
| `longhao_machine` | Longhao's machine |
| `maxwell_machine` | Maxwell's machine |
| `fsc_lab_machine` | Shared FSC Lab machine |

## Available scripts

```{toctree}
:maxdepth: 1

single_drone_sitl
single_drone_sitl_payload
single_drone_sitl_payload_test
multi_drone_sitl
three_drone_point_mass_payload_sitl
three_drone_rigid_body_payload_sitl
```
