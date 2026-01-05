## Fsc Aerial Manipulation-Pegasus-PX4 SITL Launch Scripts
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