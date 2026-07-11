## Fsc Aerial Manipulation-Pegasus-PX4 SITL Launch Scripts

> Full per-script documentation (what each one launches, tmux layout, prerequisites) lives in the Sphinx docs under `docs/source/fsc_lab/index.md`.

### 1. How to launch the simulation:
```
{$PATH_TO_FSC_PEGASUS}/scripts/start_single_drone_sitl.sh {CONFIG_FILE_NAME}
```
- For example:
```
source/fsc_PegasusSimulator/scripts/start_single_drone_sitl.sh longhao_machine
```
### 2. To setup the path for your machine, create a new .conf file in the config folder. 

- In the new .conf file, add the following:
- The directory of the PX4 SITL:
```
PX4_DIR="$HOME/PX4-Autopilot"
```
- The path to the root of fsc_Pegasus repo:
```
FSC_PEGASUS_ROOT="/home/longhao/source/fsc_PegasusSimulator"
```
- The path to Issac sim python launch script:
```
ISAAC_PY="$HOME/isaacsim/python_r_fsc.sh"
```

### 3. If a scenario crashes or closes uncleanly

The launch scripts kill each other's tmux pane (PX4 <-> Isaac Sim) when either side exits, so a
lingering half shouldn't normally happen. If it does anyway (e.g. a process started outside these
scripts), scan for and clean it up with:
```
./scripts/kill_stale_sim_processes.sh          # scan, ask before killing
./scripts/kill_stale_sim_processes.sh -y        # scan, kill without asking
./scripts/kill_stale_sim_processes.sh --dry-run # scan only
```