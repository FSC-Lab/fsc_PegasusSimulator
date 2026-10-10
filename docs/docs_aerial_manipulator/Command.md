# Aerial-Manipulator Simulation — Command Reference

## 1. AM-T650 DIRECT actuation — arm held in-process

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_am_t650_stack.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuation_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuation_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**In flight**

```bash
# enter DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: true}"

# abort back to SAFETY — keep this ready before entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: false}"

# PX4 refuses an in-air disarm, so land by reference FIRST, then:
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

## 2. AM-T650 baseline (SAFETY-only)

**fsc_lab_machine**

```bash
# terminal 1 — ROS 2 baseline stack (owns the agent; detach with Ctrl-b d)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_baseline_am_t650_stack_fused.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_baseline_t650_aerial_manipulator_stack_fused.sh fsc_lab_machine uav_0

# terminal 2 — Pegasus / PX4 SITL, lockstep ON
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_baseline_sitl.sh fsc_lab_machine
```

**shiqi_machine**

```bash
# terminal 1 — ROS 2 baseline stack (owns the agent; detach with Ctrl-b d)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_baseline_t650_aerial_manipulator_stack_fused.sh shiqi_machine uav_0

# terminal 2 — Pegasus / PX4 SITL, lockstep ON
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_baseline_sitl.sh shiqi_machine
```

**Verify before arming**

```bash
ros2 param get /uav_0/autopilot_sv_baseline_node vehicle_mass       # 3.74617
ros2 param get /uav_0/autopilot_sv_baseline_node posctl_k_vel_x     # 3.7
ros2 topic hz /uav_0/mocap                                          # ~250 Hz
```

**Arm feedforward check (hover)**

```bash
ros2 topic echo --once /uav_0/fsc_autopilot_ros2/position_controller/ude
```

## 3. AM-T650 DIRECT with the ROS 2 position-mode arm stack

**shiqi_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuation_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_am_t650_stack.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuation_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Build the arm stack — shiqi_machine**

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash && source ~/ros2_ws/rosdeps/local_setup.bash
colcon build --packages-select \
  dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \
  open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge utils_custom_ground_station \
  --symlink-install
```

**Build the arm stack — fsc_lab_machine (one-time)**

```bash
# one-time apt (needs the sudo password). pinocchio is a hard BUILD dep of
# open_manipulator_x_custom_controller; Qt6 Widgets+Charts of custom_gui.
sudo apt install ros-humble-pinocchio qt6-base-dev libqt6charts6-dev \
                 ros-humble-robot-state-publisher

mkdir -p ~/Source/Shiqi/fsc_om_ws/src
cd ~/Source/Shiqi/fsc_om_ws
# NOTE the Gao907@ in the URL — the repo is private to Gao907; without it git reuses
# the lab's LonghaoQian credential and fails with "Repository not found"
git clone https://Gao907@github.com/Gao907/fsc_open_manipulator.git src/fsc_open_manipulator
vcs import src < src/fsc_open_manipulator/workspace.repos

# MANDATORY on this machine — python3 must be /usr/bin/python3, not the fsc_isaac_env
# venv, or the build dies with "No module named 'catkin_pkg'"
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v fsc_isaac_env | paste -sd:)
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \
  open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge utils_custom_ground_station \
  --symlink-install
```

**Command the arm**

```bash
ros2 topic pub --once /uav_0/fsc_open_manipulator/arm_position_controller/target_joint_positions \
    std_msgs/msg/Float64MultiArray '{data: [0.0, 0.698, 0.698, 0.0]}'
ros2 service call /uav_0/fsc_open_manipulator/arm_position_controller/go_home std_srvs/srv/Trigger
```

## 4. AM-T650 geometric direct actuation

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws                                                 # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2 — section 3's
#    ros2_arm launcher, REQUIRED since 2026-08-15 for the live arm feedforward)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuation_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws                                                                     # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2 — section 3's
#    ros2_arm launcher, REQUIRED since 2026-08-15 for the live arm feedforward)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuation_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**In flight**

```bash
# enter GEOMETRIC DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: true}"

# abort back to SAFETY — keep this ready before entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: false}"

# PX4 refuses an in-air disarm, so land by reference FIRST, then:
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**Confirm which law is live**

```bash
ros2 topic echo --once /uav_0/fsc_autopilot_ros2/controller_type
#   → "Baseline (Safety)"  |  "Geometric Direct Actuation"      (latched)
ros2 topic echo /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/geometric_control_debug
#   → SILENT in SAFETY; publishes at 250 Hz in DIRECT
```

**Check the build carries the live arm feedforward**

```bash
strings $WS/install/fsc_autopilot_ros2/lib/fsc_autopilot_ros2/autopilot_geometric_direct_actuation_node \
  | grep -c armff        # 0 → pre-2026-08-15 build, rebuild before flying
```

**Rebuild the arm stack after a pull**

```bash
# fsc_lab_machine (workspace ~/Source/Shiqi/fsc_om_ws; no rosdeps overlay)
cd ~/Source/Shiqi/fsc_om_ws/src/fsc_open_manipulator && git pull --ff-only
cd ~/Source/Shiqi/fsc_om_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select dynamixel_interfaces open_manipulator_x_description \
  open_manipulator_x_bringup open_manipulator_x_custom_controller \
  open_manipulator_x_isaac_bridge utils_custom_ground_station --symlink-install

# shiqi_machine (workspace ~/ros2_ws since 2026-08-23, was ~/colcon_ws —
# same pull+build there after any arm-repo change, plus the ~/ros2_ws/rosdeps
# overlay sourced before building, see section 3's build blocks)
```

## 5. Bare T650 geometric direct actuation

**shiqi_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws                                                                     # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2 — the geometric stack's own launcher)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/archive/start_t650_geometric_direct_actuator_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws                                                 # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2 — the geometric stack's own launcher)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/archive/start_t650_geometric_direct_actuator_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

## 6. AM-T650 geometric + L1 adaptive

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the geometric+L1 law (terminal 3)
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**shiqi_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/ros2_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh shiqi_machine

# steps 4-6 are machine-independent — use the fsc_lab_machine block above
```

**Automated flight (+20% kf, +10 mm CoM)**

```bash
# after this section's steps 0-3, instead of driving by hand:
cd ~/Source/fsc_PegasusSimulator/docs/sim_to_real_t650/tools
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
/usr/bin/python3 l1_payload_campaign_driver.py --namespace /uav_0 \
  --hover-z 1.2 --land-z 0.35 --soak 45 --step 15 --out ../am_l1_robustness_20260824/runA_am_l1.npz
/usr/bin/python3 am_l1_robustness_metrics.py ../am_l1_robustness_20260824/runA_am_l1.npz 10.0
```

## 7. Bare T650 geometric + L1 adaptive (769 g payload)

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/archive/start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the geometric+L1 law (terminal 3)
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**Automated flight**

```bash
# after this section's steps 0-3, instead of driving by hand:
cd ~/Source/fsc_PegasusSimulator/docs/sim_to_real_t650/tools
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
/usr/bin/python3 l1_payload_campaign_driver.py --namespace /uav_0 \
  --hover-z 1.0 --land-z 0.28 --soak 45 --step 15 --out runA_769g.npz
/usr/bin/python3 l1_payload_campaign_metrics.py runA_769g.npz
```

## 8. AM-T650 whole-body direct actuation (GMO observer, torque-mode arm)

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_GMO_6D_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_GMO_6D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Arm reference check**

```bash
ros2 topic info -v /uav_0/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory
# Publisher count: 2   (arm_planner @ /uav_0/fsc_open_manipulator,
#                       whole_body_planner @ /uav_0)
# Subscription count: 1 (external_torque_controller)
ros2 topic hz  <same topic>     # 100 Hz in either mode, from one source
```

## 9. AM-T650 whole-body + L1 adaptive observer (6-D)

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF
#    ...and the arm stack (arm_planner carries the SAFETY guard's arm fold)
cd ~/Source/Shiqi/fsc_om_ws && colcon build --packages-select open_manipulator_x_custom_controller

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_6D_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the whole-body law (terminal 3).
#    SAME service as section 8 — the mode namespace is shared on purpose.
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT.
# Same effect as the drone GS's "Return to baseline" and the automatic
# 20 deg tilt watchdog: hover where it is, fold the arm home
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**shiqi_machine**

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_6D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Compare the GMO and L1 sim yamls**

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config && diff \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_direct_actuation_t650_sim.yaml) \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml)
```

**Automated campaign**

```bash
# one data point, clean slate to npz  (gmo | l1)
~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_tune_cycle.sh l1 mytag shiqi_machine

# edit a gain between runs — unknown keys are an error, comments preserved
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --show
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py omega_c_t=4.0 decompose=false

# the whole sweep back to back (~7 min per flight; restores the yaml on exit)
~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_campaign.sh shiqi_machine

# score and plot
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_metrics.py \
    ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/l1_observer_20260906/*.npz
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_plot.py \
    out.png ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/l1_observer_20260906/*.npz
```

**Joint-posture term ablation**

```bash
# reproduce (every run is a clean relaunch; ~5 min each; yaml restored on exit)
docs/docs_aerial_manipulator/archive/posture_ablation_20260909/run_ablation.sh   # Q1, PID off, plant-side kf
docs/docs_aerial_manipulator/archive/posture_ablation_20260909/run_fix.sh        # Q3, allocator-side kf
/usr/bin/python3 docs/docs_aerial_manipulator/archive/posture_ablation_20260909/score.py docs/docs_aerial_manipulator/archive/posture_ablation_20260909/l1_*.npz
```

**Arm current-loop noise model**

```bash
# knobs — env > yaml > built-in.   PEGASUS_ARM_SERVO_MODEL is now current|ideal
#   'pwm' / 'pwm_0903' are REFUSED with a message (they modelled the droop)
sim_arm_current_noise_enable: true     # in both whole-body _sim yamls
sim_arm_current_noise_a: 0.007         # A rms
sim_arm_current_noise_bw_hz: 5.0       # first-order corner
sim_arm_current_noise_seed: 0

PEGASUS_ARM_SERVO_MODEL=ideal \
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_6D_direct_actuation_sitl.sh shiqi_machine

# the campaign (two matched 16 s-hold missions; ~9 min each, clean relaunch)
docs/docs_aerial_manipulator/archive/arm_current_noise_20260909/run_noise.sh shiqi_machine
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py \
    docs/docs_aerial_manipulator/archive/arm_current_noise_20260909/*.npz \
    docs/docs_aerial_manipulator/archive/posture_ablation_20260909/l1_mission_kx32.npz

# the model on its own (no Isaac needed)
/usr/bin/python3 extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/servo_model.py
```

**Arm friction + gravity mismatch**

```bash
docs/docs_aerial_manipulator/archive/arm_mismatch_20260914/run_mismatch.sh shiqi_machine   # A, B, C
```

## 10. AM-T650 whole-body + L1, 4-D attribution

**shiqi_machine**

```bash
# 0. clean slate            (any terminal — BOTH lines, this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 2 — also starts the arm stack + arm ground station)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 3'. INSTEAD OF 3, WITH THE EE MARKER CUBE (terminal 2) — a mocap cube welded
#     into the gripper; the arm GS's EE Trajectory tab draws it as "EE (Meas)".
#     Step 2 needs nothing extra (its emulator already lists obj_0). 30 g is the
#     flown mass — do NOT take the yaml's 0.2 kg default (it cannot take off).
PEGASUS_EE_MARKER_CUBE=1 PEGASUS_EE_MARKER_CUBE_MASS=0.03 \
  ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**fsc_lab_machine**

```bash
# 0. clean slate            (any terminal — BOTH lines, this order, and as TWO
#    SEPARATE calls: kill_stale_sim_processes.sh -y kills its own invoking shell
#    (exit 144), so anything chained after it in the same call never runs)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Mirror / robustness plant, and flight replays**

```bash
# mirror (default) -- steps 2 and 3 of this section's run sequence, unchanged
WB_SIM_PROFILE=mirror ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0
WB_SIM_PROFILE=mirror ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine
# the stress plant every campaign before 2026-09-26 flew
WB_SIM_PROFILE=robustness <same two lines>
# one replay of a flight, end to end (bag + yaml restore)
docs/docs_aerial_manipulator/archive/sim2real_tuning_20260926/tools/replay.sh f18|a2|a3 <tag>
```

**Confirm what a launch will apply**

```bash
# confirm what a launch will apply, without flying (prints the banners):
( source ~/fsc_PegasusSimulator/scripts/config/shiqi_machine.conf
  WB_SIM_PROFILE=mirror; source ~/fsc_PegasusSimulator/scripts/indoor_sim/lib/am_plant_from_yaml.sh )
# score a cycle's DIRECT window:
/usr/bin/python3 docs/docs_aerial_manipulator/archive/sim2real_tuning_20260926/wallclock_20260927/score_direct.py <npz>
```

**Compare the 6-D and 4-D yamls**

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config && diff \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml) \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml)
```

**Compare the 6-D and 4-D yamls (sim and hardware pairs)**

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config
diff params_single_aerial_manipulator_whole_body_l1{,_4d}_direct_actuation_t650_sim.yaml
diff params_single_aerial_manipulator_whole_body_l1{,_4d}_direct_actuation_t650.yaml
```

**Automated campaign**

```bash
# one run of the 4-D rig (or `l1` for section 9's 6-D rig, `gmo` for section 8's):
application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
# edit the 4-D yaml between runs (unknown key = error; writes a float where the
# file holds one -- an integer literal for a double parameter kills the node at
# startup with no log line, 2026-09-17):
/usr/bin/python3 application/robotic_arm/utils/wb_l1_set_gains.py --four-d omega_x=0.25
# the 2026-09-16/17 campaign, back to back (restores the yaml on exit; a running
# copy must NOT be edited -- bash reads a script by byte offset):
docs/docs_aerial_manipulator/archive/l1_4d_20260916/run_4d.sh shiqi_machine
# score:
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py     docs/docs_aerial_manipulator/archive/l1_4d_20260916/*.npz
/usr/bin/python3 application/robotic_arm/utils/wb_compare_metrics.py docs/docs_aerial_manipulator/archive/l1_4d_20260916/*.npz
cd docs/docs_aerial_manipulator/archive/l1_4d_20260916 && /usr/bin/python3 summarize_4d.py && /usr/bin/python3 ripple_4d.py && /usr/bin/python3 ripple_4d.py --soak
# EVERY reference-tracked state, per leg, peak / rms / settled-rms:
/usr/bin/python3 traj_errors.py --csv traj_errors.csv
PYTHONNOUSERSITE=1 /usr/bin/python3 plot_4d.py --six l1_6d_A.npz l1_6d_B.npz --four l1_4d_4d_best_A.npz l1_4d_4d_best_B.npz --out compare_6d_4d.png
```

**EE circle / figure-8 missions**

```bash
WB_L1_MISSION=ee_circle  WB_L1_EE_SCALE=0.8 application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
WB_L1_MISSION=ee_figure8 WB_L1_EE_SCALE=0.8 application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
# WB_L1_MISSION unset (= standard) is the standard mission (1 m hover, x/y/yaw steps, compatible trajectory) via wb_l1_campaign_driver.py, unchanged
```

**Check an ee_traj_* edit before flying**

```bash
cd ~/ros2_ws/src/fsc_trajectory_planner/test && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/l1_4d_planner_20260917/ee_plan_probe.py \
  ~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml circle
```

**Circle gain study**

```bash
cd docs/docs_aerial_manipulator/archive/sim2real_tuning_20260926
/usr/bin/python3 tools/circle_sweep.py --baseline            # shipped vs as-flown, offline
/usr/bin/python3 tools/circle_sweep.py --gate --jobs 16      # delay scan + robustness, parallel
tools/run_gain_isaac.sh "wb_k_w=0.9,wb_l1_omega_c_t=6.0"     # 4 Isaac flights, both plants
/usr/bin/python3 tools/score_run.py data/a2.npz --real       # score a hardware circle
tools/build.sh                                               # rebuild report.html
```

**Circle tuning round 2**

```bash
cd docs/docs_aerial_manipulator/archive/circle_tune_20260927/tools
/usr/bin/python3 gen_grid.py --par 6          # reference streams (needs ROS sourced)
/usr/bin/python3 eval_grid.py --gains ../analysis/tuned_H1b.json --tag x
/usr/bin/python3 final_gate.py cma5_log.jsonl out.json "H1b=tuned_H1b.json" only-extra
/usr/bin/python3 gains_to_yaml.py ../analysis/tuned_H1b.json --replay     # WB_REPLAY_GAINS string
tools/run_isaac_set.sh analysis/<spec>.txt     # from the campaign dir; one flight per spec line
```

## 11. PS4 remote — aim → confirm → move (superseded by section 12)

**shiqi_machine**

```bash
# 0. clean slate            (any terminal — BOTH lines, this order, as TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. PS4 joystick           (terminal 1 — leave running)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=0 ns:=/uav_0

# 2. ROS 2 stack            (terminal 2)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 3 — also the arm stack + arm ground station)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 4 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. PS4 remote on / off    (terminal 4 — after take-off and whole-body DIRECT from the drone ground station)
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged std_srvs/srv/SetBool "{data: true}"
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged std_srvs/srv/SetBool "{data: false}"

# 6. abort back to SAFETY   (terminal 4 — have it ready)
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"
```

**Pad, services and tests**

```bash
# The 4-D launcher now starts joy_node + gamepad_input itself, in a tmux window
# named `joy`, if a pad is present. Manually, or on any other rig:
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=0 ns:=/uav_0

# Every button is also a service on the SAME code path (scriptable, testable
# without a display). Engage = the sticks are live; REFUSED unless the vehicle
# is in whole-body DIRECT:
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged   std_srvs/srv/SetBool "{data: true}"
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/confirm_target std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/move           std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/go_home        std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/reset_target   std_srvs/srv/Trigger {}

# Loopback test: fake planner + synthetic sticks + the real ground station,
# no Isaac, no PX4, no gamepad. 21 checks of the aim/confirm/move flow.
# ROS_DOMAIN_ID=77 runs it beside a LIVE stack without touching it (the
# stale-process guard is skipped on a non-default domain).
cd $FSC_OM_ARM_WS/src/fsc_open_manipulator/utils_custom_ground_station/test
ROS_DOMAIN_ID=77 /usr/bin/python3 test_ps4_remote_loopback.py

# Synthetic sticks alone, to drive a running station by hand:
/usr/bin/python3 joy_sim.py --ly 1.0                 # hold +X
/usr/bin/python3 joy_sim.py --pattern circle         # sweep the left stick

# VERIFY THE MAPPING ON A REAL PAD BEFORE FLYING IT. Push each stick and read
# the direction it names; nothing here commands the arm:
/usr/bin/python3 check_ps4_mapping.py
```

## 12. PS4 real-time teleoperation (whole-body L1 4-D)

**shiqi_machine**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack + 2. Pegasus -- the PS4 test yaml on BOTH (the Pegasus launcher opens the pad window)
export WB_SIM_YAML=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_ps4test.yaml
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 3. take off in SAFETY and enter whole-body DIRECT (exits once DIRECT + planner HOLD)
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up

# 4. arm station "PS4 Remote" tab: Engage Teleoperation (or the state panel's PS4 REMOTE),
#    centre the pad, fly. Release Teleoperation when done (settles, then HOLD).

# 5. land
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py land
```

## 13. Modular adaptive vs whole-body L1 on the circle

**shiqi_machine**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y
# 1. stack, 2. Pegasus (WB_SIM_PROFILE=mirror|robustness on both, default mirror)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh shiqi_machine
# 3. SAFETY takeoff, then DIRECT exactly as the whole-body rig (same service):
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# The matched comparison flight, either rig (the cycle does 0-3 itself, then the mission):
export DISPLAY=:1 AM_CMP_OUT=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/modular_adaptive_20260930/data
~/fsc_PegasusSimulator/application/robotic_arm/utils/am_compare_cycle.sh modular|wb <tag> shiqi_machine -- \
  --shape circle --radius 0.5 --lap-time 24 --laps 1 --q2-period 6 --fold-deg 55 --q2-center-deg 25 \
  --q2-amp-deg 15 --time-scale 0.48 --start-pos-tol 0.10 --gate-speed 0.10
```

## 14. Controller comparison at real time (RTF 1)

**Whole-body and modular rigs**

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/rtf_profile_20261001
./run_rt1.sh wb:<tag> modular:<tag>          # headless, pacer, pinned; PEGASUS_HEADLESS=0 for a window
```

**Geometric L1 rig on the mirror plant**

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/rtf_profile_20261001
/usr/bin/python3 tools/make_geometric_mirror_yaml.py
START_POS_TOL=0.25 WB_SIM_YAML=$PWD/variants/geometric_l1_mirror_sim.yaml ./run_rt1.sh decoupled:<tag>
```

**Geometric L1 rig, tuned gains**

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/geometric_l1_tune_20261001
/usr/bin/python3 tools/make_tuned_yaml.py --gains analysis/geo_final_gains.json
WB_SIM_YAML=$PWD/variants/geometric_l1_mirror_sim_tuned.yaml ./run_isaac.sh decoupled:<tag>
```

**Figure-8, 0.20 m/s (whole-body and tuned geometric L1)**

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/figure8_compare_20261005
./run_fig8.sh wb:<tag> decoupled:<tag>       # A 0.75 / B 0.375 m, lap 22.865 s, q2 25±15° @ 5.716 s, yaw 45°
# Hardware: rebuild fsc_trajectory_planner on the Orin (the stacks refuse an older build); the 4-D
# hardware yaml carries the figure-8 (long axis on world x at any takeoff yaw, ee_traj_fig8_world_axis).
# Arm GS Figure-8 opens on the first-flight setting: Half-Length 0.70, Half-Width 0.35, Mean Velocity 0.10, Laps 1.
# The 8 is centred on the EE hover point, 0.25 m ahead of the base. Tilt watchdog: 15° on both rigs.
ROS_DOMAIN_ID=77 /usr/bin/python3 tools/hw_planner_check.py   # what the hardware yaml plans (ROS sourced)
ROS_DOMAIN_ID=77 /usr/bin/python3 tools/hw_fig8_workflow_check.py --rig wb|decoupled --harness <gs_fig8_harness>
                                                              # planner + bridge + real arm-GS panel, no vehicle
```

**Figure-8 hardware flights, 2026-10-05 → section 2 of the report "Experiment: Free-flight Comparison"**

```bash
cd ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/wb_vs_decoupled_figure8_flight_20261005/tools
AM_NPZ=<npz dir> PYTHONNOUSERSITE=1 /usr/bin/python3 summary_data.py      # extraction: README.md
cd ../../decoupled_flight_20261002/tools && python3 make_templates.py && python3 build_summary_report.py
python3 build_artifact_page.py <out.html>                                  # https://claude.ai/artifact/LhpXomd3ooNuKquPoQ9Joq
# EE rms: whole-body 37–44 mm (0.10 m/s), 43–58 mm (0.13); decoupled 73 / 77 mm; heading 0.5° vs 3.6 / 4.7°
```

## 15. PS4 teleoperation on the decoupled rig (geometric + L1, position-mode arm)

**shiqi_machine**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (decoupled: geometric+L1 node, planner + reference bridge in the `planner` window)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 / arm stack + arm GS + the `joy` window
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_direct_actuation_sitl.sh shiqi_machine

# 3. SAFETY takeoff -> geometric+L1 DIRECT (detects the controller; exits at DIRECT + HOLD, transient settled)
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up

# 4. arm station "PS4 Remote" tab: Engage Teleoperation, centre the pad, PRESS PS FIRST (pad home), fly.
#    Release Teleoperation when done (settles, then HOLD).

# 5. land
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py land
```

## 16. Pick-and-place scene (whole-body L1 4-D and decoupled geometric + L1)

**shiqi_machine** -- the same scene, payload and task for both controllers; one block each, every
command in its own terminal. Payload (2026-10-07): the CAD box (`Box_Payload.usda`, a basket with
a wire hanger), held by a HOOK grasp -- the fingers go in under the hanger's arch on both sides of
its stem and the lift hangs the arch on them; `PEGASUS_PNP_PAYLOAD=plate` in front of step 2 flies
the old box + clamped handle. Platform (2026-10-08): the printed HAT on each 1 m pillar (`Top_Hat.usda`,
Ø200.6 mm, its top 8 mm above the pillar top; the pillars are 97.7 mm, the hat's ribs' fit);
`PEGASUS_PNP_PLATFORM=cap` in front of step 2 = the old 160 mm disc. Records: `archive/pick_place_box_payload_20261007/README.md`
(payload), `archive/pick_place_top_hat_20261008/README.md` (hat). `WB_SIM_PROFILE=pick_and_place` in step 1 is REQUIRED (the yamls end in `_sim_pick_and_place.yaml`
since 2026-10-08; `pick_place` is still accepted as an alias): step 2 refuses to start
unless the running planner and controller came from the pick-and-place yamls.

**Whole-body controller (4-D L1)**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (4-D L1 node, EKF2-fused estimator, emulator with obj_0 + drop_0, planner, drone GS)
WB_SIM_PROFILE=pick_and_place ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh shiqi_machine uav_0

# 2. the pick-and-place scene + PX4 + torque-mode arm stack + arm GS (+ gamepad if plugged in)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh shiqi_machine

# 3. take off to 1 m, hover there 3 s, switch to DIRECT (wait for "Ready for takeoff!" in the PX4 pane first).
#    Publishes the SAFETY reference on the drone GS's own topic (position_controller/reference) at the
#    vehicle's x/y, z = 1.0 -- do NOT also send a target from the drone GS while it runs. offboard -> arm ->
#    climb; the hover counts once the body has stayed within 8 cm of 1 m and below 0.1 m/s for 3 s
#    (--settle 0 adds no extra wait), then set_direct_mode(true). Exits at DIRECT + planner HOLD
#    (~22 s); then use the arm GS's Pick & Place tab. Refuses an already-armed vehicle (clean relaunch).
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up --hover-z 1.0 --settle 0
```

**Decoupled controller (geometric + L1)**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (geometric + L1 node, raw mocap odometry, emulator with obj_0 + drop_0, planner + reference bridge, drone GS)
WB_SIM_PROFILE=pick_and_place ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. the pick-and-place scene + PX4 + position-mode arm stack + arm GS (+ gamepad if plugged in)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh shiqi_machine

# 3. take off to 1 m, hover there 3 s, switch to DIRECT -- the same command as the whole-body block
#    (it finds the geometric + L1 controller by itself). Exits at DIRECT + planner HOLD.
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up --hover-z 1.0 --settle 0
```

**Modular adaptive controller (MAC) -- the comparison's third rig (2026-10-08)**

```bash
# 0. clean slate (TWO separate calls), as above
# 1. stack (modular adaptive node, raw mocap, emulator with obj_0 + drop_0, planner on the modular pick-and-place yaml)
WB_SIM_PROFILE=pick_and_place ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_modular_adaptive_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0
# 2. the pick-and-place scene + PX4 + torque-mode arm stack + arm GS (the modular launcher takes the scene hooks since 2026-10-08)
WB_SIM_PROFILE=pick_and_place AM_ISAAC_SCENE_SCRIPT=~/fsc_PegasusSimulator/application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py \
  AM_ISAAC_SCENE_LABEL=AM-T650-MODULAR-PNP PEGASUS_EE_MARKER_CUBE=0 \
  ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_modular_adaptive_direct_actuation_sitl.sh shiqi_machine
# 3. as the whole-body block (the modular node answers under the whole-body namespace)
```

**The three-controller comparison (2026-10-08)** -- `results/utils/run_pick_and_place_campaign.sh [N] [wb geo mod]`
(detached; one `archive/pick_place_top_hat_20261008/tools/run_pnp.sh` flight per attempt, clean slate between,
closes the simulation at the end), scored by `PYTHONNOUSERSITE=1 /usr/bin/python3 results/utils/build_pick_and_place.py
[--paper <main.tex>]` into `results/utils/tables/pick_and_place_sim.{csv,json,tex}`, the MATLAB files and the three
figures; record `results/simulation_results/README.md`. Since 2026-10-09 every run flies the driver's `--timetable`
(each step starts at the same mission time on every run and controller; a late step re-flies the run) and the sim
pick-and-place yaml sets the planner's `pick_place_descent_trim_first_min: 0`, so the six phases line up across runs;
MATLAB folder `results/simulation_results/matlab_simulation_data/pick_and_place/` (self-contained; its README.md).

**HARDWARE (the Orin, 2026-10-08)**: `AM_HW_PROFILE=pick_and_place` in front of either whole-body hardware stack
script (fused / raw) or either decoupled one selects the parallel `..._t650_pick_and_place.yaml` (generated by
`archive/pick_place_top_hat_20261008/tools/make_hw_pick_and_place_yamls.py`: the free-flight hardware file + the
pick-and-place gains the simulation validated -- whole-body k_R/k_w 1.6/1.2 + the anchor blend; decoupled
kp/kv 15/10, kp_z/kv_z 40/18); the scripts print `PROFILE:` and check the profile's own values. Unset = free flight.

**HARDWARE flights 2026-10-09 (whole-body, fused)** -- record `archive/wb_pick_place_flight_20261009/README.md`,
report https://claude.ai/artifact/SKoEL1B8DjJR1fLYaLpfLS. PP-1 flew all six legs without a payload (33 mm rms);
PP-2 hooked the basket (190 g) and set it down 166 mm short of the place point; PP-3 missed with
EE Offset z 0.20. From the data (revised 2026-10-10): **EE Offset [-0.02, 0, 0.16]** (the fingers meet the arch at 0.167: 0.17 puts
them at arch height for two thirds of the slide-in), **Pick = Get x + 0.01**, **Place = Get x − 0.03 (further along the
approach; check the hang on the ground with the new, wider handle first), Get y, Get z − 0.02, yaw −180**, margins unchanged;
wait ≥ 8 s in the hover before **4.2 Place**: the planner's descent trim window is 5 s since 2026-10-10
(`pick_place_descent_trim_window_s: 5.0` in the hardware whole-body `_pick_and_place.yaml`, read by both rigs' planner;
with the 1 s default it sampled the arrival swing). Open: the PX4 ↔ Orin link froze
in PP-2's last hover (2.6 m fly-away), both STAB landings tipped over -- land through SAFETY from ≥ 0.8 m.

**Operating the arm GS "Pick & Place" tab**

Before the flight (on the ground, after steps 1–2)

1. **Adjust** — with the vehicle on (or over) its start mark. The shift is written into the four drone points (Start, Place Start, Land Start, Land).
2. Basket on the place hat → **Get** on the *Place* row (2026-10-10) — fills x, y, z from the basket's mocap
   pose and keeps the typed yaw (−180). Then lower z by hand: the basket centre at release sits 2 cm below its
   resting height, so the claw ends out of the arch (sim default 1.02). An edit is written to the planner on Enter; Adjust never moves it.
3. Basket on the pick hat → **Get** on the *Pick (obj_0)* row — captures the payload box's mocap pose (x, y, z, yaw)
   into the row's boxes, which can be edited afterwards (2026-10-10). Both Gets refuse a frozen pose (body not
   tracked) and a yaw that flips inside the half-second window.
4. **EE Offset** — type the three boxes, press the button (default [-0.02, 0, 0.16] m: the claw
   just past the hanger's stem, its fingers under the arch). The claw
   targets (green, *Planned goal*) = Pick / Place pose + this offset turned by its yaw.
5. **Vertical Margin** — the clearance above both EE targets (default 0.20 m), and **Side Margin** —
   how far behind the stem Ready To Pick hovers and how far Exit To Place backs out (default
   0.10 m, hook grasp only); each: type, press its button. Both sit right of the points.

During the flight (after step 3: DIRECT, planner HOLD)

1. **Plan** — dry-runs all six legs and draws the path. Wait for `READY next=go_to_start`.
2. **1 Go To Start**, then **2.1 Ready To Pick** — opens the gripper and hovers the Side Margin
   behind the hanger's stem at grasp height (`WAITING … press Pick to descend`).
3. **2.2 Pick** — slides in level under the arch, and closes the gripper once the EE is within
   2 cm AND centred to 3 mm (the lamp shows it). The fingers close AROUND the 3 mm stem (they
   cannot clamp it), so a full close is the grasp: **2.3 Exit To Pick** then starts by itself
   (up the vertical margin, 3.2 s; the arch settles on the fingers).
4. **3 Go To Place Start** (turns clockwise to −180°, arm to the carry pose), **4.1 Ready To
   Place**, **4.2 Place** (descends with the jaws CLOSED; at the bottom the gripper opens and
   **4.3 Exit To Place** starts by itself: the Side Margin back out from under the arch, then up the
   vertical margin, then the gripper closes — when the basket lands the vehicle lurches ~10 cm back (decoupled: ~20 cm) and returns, so do not wait for the claw).
   Arm poses: pick / place [0, 32, 38, 0], carry [12, 38, 42, 0]. The hook needs grip on the
   fingers' top edges: in Isaac it slips off at friction 0.4.
5. **5 Go To Land Start** (on clockwise to −360°), **6 Execute To Land** — hovers over the
   landing spot (`COMPLETE`). Touch down:
   ```bash
   FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
     /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py land
   ```

- **ABORT** (red, any time, highest priority): opens the gripper; the vehicle climbs 0.30 m while
  the arm goes to the RELEASE pose [0, 0, 30, 0] (claws 60° down: an open gripper still carries the
  hook's arch, so the basket must slide off), holds it 2 s, then folds home; the gripper closes
  when the abort is complete (~10.5 s), and the vehicle hovers. Isaac 2026-10-09: the basket was
  released in all three cases -- mid-carry it falls to the floor; hooked on a hat it is lifted
  ≤14 mm and stays there upright. Then re-fly the cut leg or press **Reset** (clears the plan and
  progress, keeps Get / Adjust / typed points). The planner's **safety guard** does the same by
  itself if the tilt stays above 15° for 0.1 s -- but on hardware the flight node's own 15° tilt
  watchdog trips first and reverts to SAFETY (hover in place, arm home after 1 s, gripper unchanged).
- Steps only enable in order; any leg already flown can be flown again.
- Optional fine correction: once a claw leg is `DONE` and green, the **Pick & Place PS4** tab →
  **Engage**; the sticks nudge the claw, **R1** closes and **L1** opens the gripper. Pressing the next
  leg disengages it.


## 17. Push-and-pull scene (whole-body L1 4-D, contact phase)

**shiqi_machine** -- the HOOBRO console table (1.000 x 0.150 x 0.800 m) in front of the start, its length
along y; the real 400 g box near its +y end, from the user's CAD (`Box_Push.stl` -> `Box_Push.usda`, 2026-10-07:
240 x 160 x 100 mm, friction 0.29 = the measured experiment table) with its clamp bracket and a 5 mm fin
handle on the top centre (grasp 0.233 m above the table; the sim claw cannot close below ~19 mm, so the
fin's bare part carries an invisible 20 mm grip shim, `PEGASUS_PUSH_FIN_GRIP_MM`). `PEGASUS_PUSH_HANDLE=post`
gives the 10-06 post (grasp 0.273 m; printable: `archive/push_pull_20261003/handle/`). The airframe carries the T650's landing gear, 7 cm shorter than
the X650's (`AM_T650.usda`, a copy of `AM_xfwd.usda`; planner `push_pull_gear_depth` 0.2425); the
vehicle grasps the post and slides the box 0.50 m along the table with the arm held still (folded forward,
[0, 30, 40, 0] deg). The law's contact phase flag follows the GRIPPER: CONTACT from the jaws' grip (the
Push press) to the end of the push, before the jaws open, and
the claw stays held in the world through the push. Flown 2026-10-04: 3 / 3 complete missions, the box
pushed 495-502 mm and left on the table. `WB_SIM_PROFILE=push_pull` in step 1
is REQUIRED: step 2 refuses to start unless the running planner and controller came from the
push-and-pull yaml. Record: `docs/docs_aerial_manipulator/archive/push_pull_20261003/`; the CAD box:
`archive/push_pull_cad_box_20261007/`.

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (4-D L1 node, RAW mocap feedback, emulator with obj_0, planner, drone GS). Not the _fused
#    stack: in contact EKF2 drifts 1-1.5 cm / 1-3 deg and every fused push failed (2026-10-05, record section 8)
WB_SIM_PROFILE=push_pull ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. the push-and-pull scene + PX4 + torque-mode arm stack + arm GS (+ gamepad if plugged in)
PUSH_FEEDBACK=raw ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_push_and_pull_sitl.sh shiqi_machine

# 3. take off to 1.3 m, hover 3 s, switch to DIRECT (wait for "Ready for takeoff!" in the PX4 pane first)
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up --hover-z 1.3 --settle 0
#    (1.3 m, not 1.0: the push pose holds the body lower over the box; from 1.0 m Plan refuses Go To Start)
```

Or the whole mission scripted, headless (after step 0): `archive/push_pull_20261003/tools/run_pl.sh <tag>`.
Score it (impedance residual + rho_UAV): `PYTHONNOUSERSITE=1 /usr/bin/python3 archive/push_pull_20261003/tools/pl_metrics.py runs/<tag>.npz`.

**Operating the arm GS "Push & Pull" tab**

Before the flight (on the ground, after steps 1-2)

1. **Adjust** -- the vehicle on its start mark; the shift is written into the Start mark and Land.
2. **Get** on the *Box (obj_0)* row -- captures the box's mocap pose (x, y, z, yaw).
3. **EE Offset** (sim [0, 0, 0.183] m for the CAD box, box frame), **Safety Margin** (0.20 m), **Push Distance**
   (0.50 m; negative pulls) -- type, press the button.

During the flight (after step 3: DIRECT, planner HOLD)

1. **Plan** -- dry-runs all five steps and draws the path. Wait for `READY next=go_to_start`.
2. **1 Go To Start** -- to the table's left end, turning right to -90 deg; the claw 0.12 m behind and
   0.10 m above the post's grasp point.
3. **2 Ready To Push** -- opens the gripper and slides diagonally down-forward onto the handle (CoM-anchored;
   the claw is world-held from its arrival).
4. **3 Push** -- closes the gripper once the claw is within 2 cm AND centred across the post to 3 mm;
   once the jaws grip, the law goes to CONTACT (the contact line turns purple) and the box slides
   0.50 m (1.5 s settle + 12 s); CONTACT ends with the push. If the gripper closes on nothing,
   nothing moves.
5. **4 Exit To Push** -- opens the gripper, waits 1 s, up 0.20 m (the gear clears the box), arm home.
6. **5 Go To Land** -- on clockwise to -360 deg, a hover 0.5 m behind the start (`COMPLETE`).
   Touch down with `ps4_teleop_bringup.py land --land-z 0.25` (section 16; the T650 gear rests the body at
   0.235 m -- the default 0.35 would leave it hovering 11 cm up).

- **ABORT** (red, any time): gripper open, contact off; on the handle a 0.5 s hold for the jaws,
  then 0.30 m up and the arm home. After an abort with the box touched: **Get** and **Plan** again.
  The planner aborts by itself if the tilt stays above 15 deg for 0.1 s, or the push force the
  law reads stays above 5 N for 0.3 s.
- Scene knobs (env, before step 2): `PEGASUS_PUSH_BOX_MASS`, `PEGASUS_PUSH_FRICTION_STATIC` /
  `_DYNAMIC` (0.29 / 0.29 with 400 g: 1.14 N to break the box loose and to slide it -- keep the two EQUAL: at
  0.7 / 0.6 the box sticks and then lets go, and the vehicle surges), `PEGASUS_PUSH_BOX_XY`,
  `PEGASUS_PUSH_Q2_LIMIT_DEG` (default `-20,45`, the real arm's q2 range as hard stops; `asset` = -90,50).
  Inside that range the shipped 70 deg pose fails; only the 50 deg pose [0, 26, 24, 0] with
  `wb_l1_omega_c_t` 1.0 pushed the full 0.50 m (2 / 2), and no pull worked (record section 9.2). CAD box
  (2026-10-07): the push yaml now flies the 60 deg pose [0, 26, 34, 0], `wb_l1_omega_c_t` 1.0,
  `push_pull_push_time_s` 24, and a CoM-anchored hover and descent with the claw world-held only from its
  arrival on the handle -- 3 / 3 (12 s pulls: 0 / 3). Still open: the release folds the arm onto q3's +50 stop
  (every pull, most pushes) and the exit can drag the box. Don't pull on hardware.
