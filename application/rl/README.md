# Pegasus RL Hover Baseline

This folder documents the PPO-based hover training workflow built **without Isaac Lab**.

It uses:

- Pegasus Simulator
- Isaac Sim
- Gymnasium
- Stable-Baselines3
- TensorBoard

## Relevant files

### `application/rl/rl_backend.py`
Receives the RL action and forwards motor references to the Pegasus multirotor backend.

### `application/rl/hover_env.py`
Defines the hover task environment:

- action space
- observation space
- reward function
- termination conditions
- success definition

### `application/rl/train_hover_ppo.py`
Starts Isaac Sim in training mode, creates the Pegasus world and multirotor, runs PPO training, and saves:

- checkpoints
- final trained policy
- TensorBoard logs

### `application/rl/eval_hover_policy.py`
Loads a saved PPO policy and runs it with rendering enabled for visualization.

---

## Dependencies for the current non-Isaac-Lab workflow

### 1. System packages
```bash
sudo apt update
sudo apt install -y git cmake build-essential
```

### 2. Isaac Sim path
```bash
export ISAACSIM_PATH="$HOME/workspaces/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64"
export ISAACSIM_PYTHON_EXE="$ISAACSIM_PATH/python.sh"
```

You can make them persistent:
```bash
echo 'export ISAACSIM_PATH="$HOME/workspaces/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64"' >> ~/.bashrc
echo 'export ISAACSIM_PYTHON_EXE="$ISAACSIM_PATH/python.sh"' >> ~/.bashrc
source ~/.bashrc
```

### 3. Python packages
If using Isaac Sim bundled Python:
```bash
$ISAACSIM_PATH/python.sh -m pip install --upgrade pip setuptools wheel
$ISAACSIM_PATH/python.sh -m pip install gymnasium stable-baselines3 tensorboard scipy numpy
```

If using your Isaac-compatible virtual environment instead:
```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install gymnasium stable-baselines3 tensorboard scipy numpy
```

### 4. Project environment variables
```bash
export PYTHONPATH="$HOME/fsc_PegasusSimulator:$PYTHONPATH"
export OMNI_KIT_ACCEPT_EULA=YES
```

---

## How to train

```bash
cd ~/fsc_PegasusSimulator
$ISAACSIM_PATH/python_r.sh ~/fsc_PegasusSimulator/application/rl/train_hover_ppo.py
```

---

## What gets saved

Typical generated artifacts:

- `checkpoints/`
- `ppo_hover_policy_final.zip`
- `ppo_hover_tensorboard/`

These are generated outputs and usually **should not be committed to Git**.

---

## How to launch TensorBoard

```bash
tensorboard --logdir ~/fsc_PegasusSimulator/ppo_hover_tensorboard --port 6006
```

Then open:

```text
http://localhost:6006
```

If using SSH from another machine, forward the port:

```bash
ssh -L 6006:localhost:6006 <user>@<host>
```

---

## How to render / evaluate

```bash
cd ~/fsc_PegasusSimulator
$ISAACSIM_PATH/python_r.sh ~/fsc_PegasusSimulator/application/rl/eval_hover_policy.py
```

This launches Isaac Sim with rendering enabled and runs the saved hover policy.

---

## Suggested Git practice

Commit source code and docs:

- `application/rl/`
- `application/rl/README.md`
- `.gitignore`
- any machine config you intentionally want to keep, such as `scripts/config/markz_machine.conf`

Do **not** commit generated artifacts by default:

- `checkpoints/`
- `ppo_hover_tensorboard/`
- `ppo_hover_policy_final.zip`
- `__pycache__/`
