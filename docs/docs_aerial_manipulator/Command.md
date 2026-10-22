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

**shiqi_machine** -- the same scene and task for both controllers; one block each, every command in
its own terminal. TWO DEMONSTRATIONS (2026-10-14), one `WB_SIM_PROFILE` each, in front of BOTH step 1
and step 2:
- `pick_and_place` (the paper's comparison): the 10-07 one-piece basket + wire hanger (`Box_Payload.usda`),
  1.0 m pillars, the basket SET DOWN on the place hat; arm GS tab **Pick & Place**.
- `pick_and_drop`: the real payload (`Box_Assembly.stl` -> `Box_Payload_Assembly.usda`: basket, the
  holder without its arch, 5 mocap markers, 180 g), the lab's 0.77 m pillars, and a transparent
  371 x 290 mm bin, walls cut to 35 mm (2026-10-15, was 172), on the place hat, turned so its length runs along y. Clockwise route: pick ->
  Place Start (0.8, -1, 1.0) -> Ready To Place flies straight to the drop point over the bin, yaw -180
  (like the set-down's Ready To Place; `pick_place_drop_climb_lead` 1.0) -> Place opens the gripper ->
  Exit To Place starts by itself once the gripper reports open, from where the drone is, backs out the
  Back Margin (+x) and climbs the Vertical Margin, and holds there -> Go To Land Start folds the arm and
  closes the gripper -> Land Start (-0.5, 0.8, 1.0) -> Land 0.35; arm GS tab **Pick & Drop** (it names the
  place steps Drop Start / Ready To Drop / Drop / Exit To Drop, and the wrapper opens the arm GS on it). The drop's height (2026-10-15): the Drop
  row = the payload standing on the bin's cover (Get, no hand correction), and the **Drop Margin** box
  beside it (`pick_place_drop_height`, preset 0.20 m) = how far above that the gripper opens;
  `pick_place_drop_altitude` is 0. The **Back Margin** box under it (`pick_place_drop_exit_back`,
  preset 0.20 m) = how far Exit To Drop backs out before it climbs.
Hardware: `AM_HW_PROFILE=pick_and_drop` (the drop block, `..._t650_pick_and_drop.yaml`) or
`AM_HW_PROFILE=pick_and_place` (the set-down that flew 10-09/10-10) on the whole-body stack.
Attitude gains: pick-and-place k_R/k_w 1.6/1.2; pick-and-drop the free-flight 2.134/1.567 since 2026-10-16,
sim and hardware (smaller lift / release lurch, `archive/lift_lurch_gains_20261016/README.md`).
Wrist roll j4 (hardware arm yaml, 2026-10-16): `friction_ff[3]` 4.455 + `passthrough_error_damping_joints`
[0, 0, 0, 0.05] (`archive/q4_damping_20261016/README.md`); rebuild `open_manipulator_x_custom_controller`
on the Orin -- an old build ignores the new key (the whole-body stacks warn).
Arm torque caps widened for flight (2026-10-16): `max_effort` [85, 420, 260, 57.6] duty = [0.50, 2.81, 1.92,
0.39] N.m; the hardware arm launch derives the servo PWM Limits from them ([212, 587, 414, 189], printed at
bring-up). `archive/arm_caps_20261016/README.md`; same Orin rebuild.
The profile also sets the scene (07's `PEGASUS_PNP_TASK` place / drop); `PEGASUS_PNP_PAYLOAD`,
`PEGASUS_PNP_PICK_PILLAR_HEIGHT` / `PEGASUS_PNP_PLACE_PILLAR_HEIGHT`, `PEGASUS_PNP_BIN=0|1` in front of
step 2 still override it. The CAD payloads are clamped in the sim on an invisible 20 mm grip pad
(`PEGASUS_PNP_FIN_GRIP_MM=0` = none). Platform (2026-10-08): the printed HAT on each pillar
(`Top_Hat.usda`, its top 8 mm above the pillar top); `PEGASUS_PNP_PLATFORM=cap` = the old 160 mm disc.
Records: `archive/pick_place_drop_20261014/README.md` (the two demonstrations, the drop),
`archive/pick_place_box_payload_20261007/README.md`, `archive/pick_place_top_hat_20261008/README.md`.
Step 2 refuses to start unless the running planner and controller came from that profile's yaml
(`_sim_pick_and_place.yaml` / `_sim_pick_and_drop.yaml`; `pick_place` = an alias of `pick_and_place`).

**Whole-body controller (4-D L1)**

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (4-D L1 node, EKF2-fused estimator, emulator with obj_0 + drop_0, planner, drone GS)
WB_SIM_PROFILE=pick_and_place ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh shiqi_machine uav_0

# 2. the pick-and-place scene + PX4 + torque-mode arm stack + arm GS (+ gamepad if plugged in)
WB_SIM_PROFILE=pick_and_place ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh shiqi_machine

# 3. take off to 1 m, hover there 3 s, switch to DIRECT (wait for "Ready for takeoff!" in the PX4 pane first).
#    Publishes the SAFETY reference on the drone GS's own topic (position_controller/reference) at the
#    vehicle's x/y, z = 1.0 -- do NOT also send a target from the drone GS while it runs. offboard -> arm ->
#    climb; the hover counts once the body has stayed within 8 cm of 1 m and below 0.1 m/s for 3 s
#    (--settle 0 adds no extra wait), then set_direct_mode(true). Exits at DIRECT + planner HOLD
#    (~22 s); then use the arm GS's Pick & Place tab (Pick & Drop for pick_and_drop). Refuses an already-armed vehicle (clean relaunch).
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
WB_SIM_PROFILE=pick_and_place ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_pick_and_place_sitl.sh shiqi_machine

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
   resting height, so the claw ends out of the arch (sim default 0.87, the 10-10 flight's). An edit is written to the planner on Enter; Adjust never moves it.
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
- **Pick & Drop** tab (2026-10-14; it replaced the Pick & Place PS4 tab): the same steps, but 4.1 Ready To
  Place climbs at Place Start and holds over the bin, the Drop Margin above the Drop point, 4.2 Place opens the gripper
  there and 4.3 Exit To Place then starts by itself: back the Back Margin, up the Vertical Margin, hold;
  5 Go To Land Start folds the arm and closes the gripper. The arm holds the pick pose [0, 32, 38, 0]
  from the grasp to the drop (carry pose = pick pose since 2026-10-16; the set-down keeps [12, 38, 42, 0]).
  AUTONOMOUS DROP (2026-10-16, `pick_place_drop_auto`): after Ready To Drop the planner releases by itself
  once, for 0.5 s, the claw is within 20 mm and still, obj_0 is within 25 mm of the Drop point and still,
  and q4 is level -- no earlier than 2 s after the leg; the status line names the gate it waits on, and
  Drop releases at once by hand. Needs the 2026-10-16 planner AND arm GS.
  AIM AT THE BASKET (2026-10-16, `pick_place_drop_aim_payload`): **hover >= 6 s at Drop Start before
  Ready To Drop** -- the planner measures where the basket hangs from the claw there and moves the drop so
  the basket, not the claw, lands on the Drop point (its reply says `aimed ...` or `NOT aimed: <why>`);
  aimed, the claw only has to be still.
  A tab's Plan switches the planner to its own place; the other tab then says so and stays disabled.


## 17. Box-pushing scene (whole-body L1 4-D, contact phase)

**shiqi_machine** -- the HOOBRO console table (1.000 x 0.150 x 0.800 m) 1.20 m in front of the start mark
(-0.60, 0), its length along y, the reference leg's outer corner at (0.60, 0.50) (2026-10-21: the whole layout
moved 0.60 m back, it was (1.20, 0.50) with the start mark at the origin); the 500 g box (2026-10-16: the box-pushing campaign's choice, total incl. the handle) near its +y end (centre (0.675, 0.28): back face 0.10 m from that end, pushed 0.50 m to 0.16 m from the far end -- the user's bench layout, 2026-10-17), from the user's CAD (`Box_Push.stl` -> `Box_Push.usda`, 2026-10-07:
240 x 160 x 100 mm, friction 0.25 = the real box on the experiment table, measured 2026-10-17) with its clamp bracket and a 5 mm fin
handle on the top centre (grasp 0.233 m above the table; the sim claw cannot close below ~19 mm, so the
fin's bare part carries an invisible 20 mm grip shim, `PEGASUS_PUSH_FIN_GRIP_MM`). `PEGASUS_PUSH_HANDLE=post`
gives the 10-06 post (grasp 0.273 m; printable: `archive/push_pull_20261003/handle/`). The airframe carries the T650's landing gear, 7 cm shorter than
the X650's (`AM_T650.usda`, a copy of `AM_xfwd.usda`; planner `push_pull_gear_depth` 0.2425); the
vehicle grasps the fin and slides the box 0.50 m along the table with the arm held still (folded forward,
[0, 32, 28, 0] deg since 2026-10-16; [0, 26, 34, 0] before). The law's contact phase flag follows the GRIPPER: CONTACT from the jaws' grip (the
Push press) to the end of the push, before the jaws open, and
the claw stays held in the world through the push. Flown 2026-10-04: 3 / 3 complete missions, the box
pushed 495-502 mm and left on the table. `WB_SIM_PROFILE=box_pushing` in step 1
is REQUIRED: step 2 refuses to start unless the running planner and controller came from the
box-pushing yaml. Record: `docs/docs_aerial_manipulator/archive/push_pull_20261003/`; the CAD box:
`archive/push_pull_cad_box_20261007/`.

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (4-D L1 node, RAW mocap feedback, emulator with obj_0, planner, drone GS). Not the _fused
#    stack: in contact EKF2 drifts 1-1.5 cm / 1-3 deg and every fused push failed (2026-10-05, record section 8)
WB_SIM_PROFILE=box_pushing ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. the box-pushing scene + PX4 + torque-mode arm stack + arm GS (+ gamepad if plugged in)
PUSH_FEEDBACK=raw ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_box_pushing_sitl.sh shiqi_machine

# 3. take off to 1.3 m, hover 3 s, switch to DIRECT (wait for "Ready for takeoff!" in the PX4 pane first)
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up --hover-z 1.3 --settle 0
#    (1.3 m, not 1.0: the push pose holds the body lower over the box; from 1.0 m Plan refuses Go To Start)
```

Or the whole mission scripted, headless (after step 0): `archive/push_pull_20261003/tools/run_pl.sh <tag>`.
Score it (impedance residual + rho_UAV): `PYTHONNOUSERSITE=1 /usr/bin/python3 archive/push_pull_20261003/tools/pl_metrics.py runs/<tag>.npz`.

**Raw-mocap guard (2026-10-21)** -- for box pushing the raw stack runs the estimator with `mocap_guard:=true`. A frozen,
jumped or stopped drone mocap switches the odometry to EKF2 + the offset frozen at the last good sample; after 0.15 s with
the claw on the box the planner aborts (release, climb, arm home) and the vehicle hovers; every step waits for the mocap,
and the arm GS's Mocap lamp reads LOST -- LAND WITH THE RC. Inject a fault in the sim:
`ros2 param set --no-daemon --spin-time 5 /isaacsim_mocap_emulator fault_type freeze` (`fault_duration_s`, default 1 s),
or `pl_mission.py --mocap-fault freeze@push+2:3`. Record: `archive/mocap_safety_guard_20261021/`.
Hardware hover test: `ros2 service call /uav_0/state_estimator/mocap_guard/test_fault std_srvs/srv/Trigger` (0.5 s:
CONFIRMED and back, never LOST). Rebuild before flying: Orin `fsc_drone_state_estimator_ros2` (branch `dev_robotic_arm`),
`fsc_trajectory_planner`, `fsc_autopilot_ros2`; laptop `utils_custom_ground_station`.

**Operating the arm GS "Box Pushing" tab**

Before the flight (on the ground, after steps 1-2)

0. **Layout (2026-10-22: the table is placed from the box)** -- put the table where the floor is flat; the box at
   its LEFT end (seen from the vehicle), centred across it, its back face 0.10 m in from the end; the vehicle on
   the table's middle line, facing it, ~1.6 m in front (at least 0.75 m). Isaac places all three a little off
   by default (`PEGASUS_PUSH_LAYOUT=field`; `nominal` = the exact 10-21 layout, which the campaign drivers use). Plan refuses a start more than 0.30 m off the
   middle line, and Go To Start flies round the table's box end (body >= 0.70 m from it, seen from above).
1. **Adjust** -- the vehicle where it will take off (since 2026-10-22 anywhere: the start, Land Start and Land
   become where it is; the landing yaw, -180, is relative to the box -- 90 deg clockwise from the push).
2. **Get** on the *Box (obj_0)* row -- captures the box's mocap pose (x, y, z, yaw); the boxes are editable.
3. **EE Offset** (sim [0, 0, 0.183] m for the CAD box, box frame), **Vertical Margin** (0.15 m), **Side Margin**
   (0.12 m), **Push Distance** (0.50 m) -- type, press the button. The **Monitor** under the points shows
   the push time, contact time and mean speed it gives (amber until sent): the time is the distance at the
   planner's `push_pull_push_mean_speed`, the one setting (yaml).

During the flight (after step 3: DIRECT, planner HOLD)

1. **Plan** -- dry-runs all six steps and draws the path. Wait for `READY next=go_to_start`.
2. **1 Go To Start** -- to the table's left end, turning right to -90 deg; the claw 0.12 m behind and
   0.15 m above the fin's grasp point.
3. **2 Ready To Push** -- starts by itself 1 s after `DONE go_to_start` (2026-10-21; press it to go sooner):
   opens the gripper and, once it reports open, slides diagonally down-forward onto the
   handle (CoM-anchored; the claw is world-held from its arrival).
4. **3 Push** -- starts by itself at `DONE ready` (2026-10-21): closes the gripper once the claw is within 2 cm AND centred across the fin to 3 mm;
   once the jaws grip, the law goes to CONTACT (the contact line turns purple) and the box slides
   the Push Distance (1.5 s settle + the push time); the Monitor shows the box's travel and speed live,
   then the push's numbers. CONTACT ends with the push. If the gripper closes on nothing,
   nothing moves (Isaac: the grasp is a stall; hardware, 2026-10-19: a full close on the fin is the
   grasp, a close stopping wider than -0.005 m a wrong grip).
5. **4 Exit To Push** -- starts by itself 0.2 s after `DONE push` (2026-10-21; press it to go sooner, ABORT to stop
   it): releases, opens the gripper and, on its OPEN report, climbs 0.25 m (the gear clears the box), arm home.
6. **5 Go To Land Start** -- (2026-10-22) round the table's RIGHT (far) end, Go To Start's mirror image: on along
   the push line at the exit height, over the pushed box (the gear kept 0.05 m over the fin's top), to 0.70 m
   past the far end, then across and back along the table's front to Land Start (1.3 m over the take-off spot),
   turning 90 deg clockwise to yaw -180 (relative to the box; ~25 s). Until 2026-10-22: one transition across
   the table, 270 deg clockwise to -360, ~24 s.
7. **6 Land** -- straight down to the hover over the start mark, where it took off (`COMPLETE`).
   Touch down with `ps4_teleop_bringup.py land --land-z 0.25` (section 16; the T650 gear rests the body at
   0.235 m -- the default 0.35 would leave it hovering 11 cm up).

- **ABORT** (red, any time): gripper open, contact off; on the handle it holds the MEASURED pose 1.0 s
  (hardware 1.5) for the jaws, backs 3 cm off against the push, then 0.30 m up and the arm home. **Then press Go To
  Land Start and Land -- never land in place: the abort leaves the vehicle beside the table.** To
  push again: **Get** and **Plan**. The planner aborts by itself if the tilt stays above 15 deg (hardware 10) for
  0.1 s, the force the law reads on the claw stays above 5 N for 0.3 s (from the grasp to the exit), the
  arm's joint states go silent for 0.5 s, and (2026-10-16) if the BOX is
  more than 40 mm behind / ahead of its plan (2026-10-21: the plan plus the law's free-axis shift) or tilts more than 2 deg (0.2 s), its mocap goes
  silent for 0.5 s, or q3 drops below 10 deg; Plan refuses a push that would leave the box
  within 3 cm of the table's edge (a box turned 180 deg, a too-long distance). A guard trip
  names itself in the status line.
- Scene knobs (env, before step 2): `PEGASUS_PUSH_BOX_MASS`, `PEGASUS_PUSH_FRICTION_STATIC` /
  `_DYNAMIC` (0.25 / 0.25 with 500 g: 1.23 N to break the box loose and to slide it -- keep the two EQUAL: at
  0.7 / 0.6 the box sticks and then lets go, and the vehicle surges), `PEGASUS_PUSH_BOX_XY`,
  `PEGASUS_PUSH_Q2_LIMIT_DEG` (default `-20,45`, the real arm's q2 range as hard stops; `asset` = -90,50).
  Inside that range the shipped 70 deg pose fails; only the 50 deg pose [0, 26, 24, 0] with
  `wb_l1_omega_c_t` 1.0 pushed the full 0.50 m (2 / 2), and no pull worked (record section 9.2). CAD box
  (2026-10-07): the push yaml now flies the 60 deg pose [0, 26, 34, 0], `wb_l1_omega_c_t` 1.0,
  `push_pull_push_time_s` 24, and a CoM-anchored hover and descent with the claw world-held only from its
  arrival on the handle -- 3 / 3 (12 s pulls: 0 / 3). Still open: the release folds the arm onto q3's +50 stop
  (every pull, most pushes) and the exit can drag the box. Don't pull on hardware.

**2026-10-09 -- PUSH is the task, the push yaml is TUNED, and the results campaign flew** (record:
`archive/push_pull_tune_20261009/README.md`; results: `results/simulation_results/README.md`
"Box pushing"). A 3 + 3 A/B at the shipped settings completed both ways, but the pull ends LOADED
(release residual 2-4.8 N rms, peaks 16 N, tilt 5-7 deg, q2 on its stop, the box dragged 3-6 mm at
every exit): only the push goes to simulation and experiment. Tune (16 screening + 8 confirmation
flights, scored on e_imp + rho_UAV + the joint margins): the EE impedance is the one lever --
`wb_ky_x/y/z` 211.9 -> **120**, `wb_dy_x/y/z` 26.82 -> **20.2** (residual halved on every window,
release peaks 0.8-1.8 N vs 3-5, the arm OFF q3's +50 stop at the release, the box untouched),
`wb_l1_omega_x_t/r/q` 0.2072 -> **0.1** (the most consistent combination; alone it does nothing). Both
observers, the contact reading (1.0 / 4.0 BOTH make the release a 16-20 N jolt), attitude, position
and the push time measured and left. The release fold onto q3's stop is GONE on this tune (42-47 deg).
`pl_mission.py --timetable` (slots 11 / 2.5 / 7 / 27 / 3 / 8.5 s) + `x_cd_ddot` / `b1_de` in the
recorded reference.

```bash
# the results campaign (3 runs, closes the sim at the end) and the MATLAB export
setsid nohup ~/fsc_PegasusSimulator/results/utils/run_box_pushing_campaign.sh 3 \
    > ~/fsc_PegasusSimulator/results/simulation_results/box_pushing/campaign.out 2>&1 < /dev/null &
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/fsc_PegasusSimulator/results/utils/build_box_pushing.py
# one tune flight: <tag> key=val ... (push_pull_* -> the planner block, wb_* / observer keys -> the law)
bash ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_tune_20261009/tools/fly.sh t_x omega_c_t=1.5
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_tune_20261009/tools/tune_table.py
```
Results (3/3, mean): e_imp slide 0.66 N rms (max 1.39), contact 0.69, release 0.89; F_ext 1.16 N;
eps_UAV 36.8 max / 17.5 rms mm (rho 0.100 / 0.047 -- the max is the turn to the landing hover, in the
push 23-37 mm); EE 18.2 mm (free-flight wander; in the slide 10 mm = F / K_y), heading 2.1 deg; box
492-498 mm, 0-0.5 mm moved in the exit, tilt <= 3.1 deg.

**2026-10-17 -- the results campaign re-flown on the 10-16 final setup** (same commands; the script
now flies `archive/box_pushing_campaign_20261016/tools/fly.sh`: 500 g, pose [0, 32, 28, 0], guards on,
the eight-step mission). 3 kept of 4 attempts: e_imp slide 0.73 N rms (max 1.44), contact 0.75;
eps_UAV 48.1 max / 16.8 rms mm; EE 17.9 mm; box 489-494 mm. **The failed attempt crashed in the
release window**: the jaws opened, the base drifted 27 mm back, q2 hit its stop, q3 went through 0,
the q3 guard tripped 1.16 s after the opening and the vehicle flipped. The 10-09/10 runs are in
`results/simulation_results/box_pushing/previous_20261009_10/`.

**2026-10-17 -- the release-window fix** (record: `archive/box_pushing_release_20261017/README.md`).
Exit To Push now RELEASES first (planner `push_pull/release`: the claw's reference back to the
CoM anchor, contact off), opens the gripper only if that succeeds, and starts the exit on the
gripper's OPEN report (was: open, exit 1 s later). 6/6 Isaac missions: q3 >= 25.8 deg, q2 <= 36.8
deg, box moved <= 1.6 mm in the exit. Rebuild `fsc_trajectory_planner` (Orin) and
`utils_custom_ground_station` (laptop); the box-pushing hardware stack refuses an older planner.
Sim driver: `pl_mission.py --release-at-open`.

**2026-10-16 -- box mass, distance, settings and safety guards** (record:
`archive/box_pushing_campaign_20261016/README.md`). 500 g (box + handle; weigh it to +-50 g --
600 g rides the joint stops), 0.50 m, the push pose re-split to [0, 32, 28, 0] (the same fold;
q3 kept off its +50 stop), the 10-09 gains unchanged (the j4 heading fix and K_y 150 were flown
and rejected), and the box guards above -- in Isaac a jammed box made the arm buckle and the
vehicle flip before the 5 N force guard tripped; with the guards every worst case flown aborts
safely. Hardware: `AM_HW_PROFILE=box_pushing` on the whole-body stack flies
`..._t650_box_pushing.yaml` (generated by the campaign's `make_hw_box_pushing_yaml.py`; its
`push_pull_table` is the SIMULATION's -- measure the lab table first; SUPERSEDED 2026-10-22: the table
is placed from the box, see step 0 above). Rebuild fsc_trajectory_planner (the guards) and the arm GS.

**2026-10-17 -- a constant-speed push was tried and is NOT used** (record:
`archive/push_cruise_20261017/README.md`). New planner option `push_pull_push_ramp_s` (0 = the
min-snap push, shipped): ramps, a constant cruise, ramps, inside `push_pull_push_time_s`. At 20 s /
3 s ramps (29.4 mm/s) the box stopped far less, but 2 of 3 pushes ended in a q3-guard abort after the
jaws opened, one a crash; the min-snap push flew 2 / 2 the same day. Keep the ramp at 0. The window
between the jaws opening and the exit (the open claw still held in the room) is a hazard of the
min-snap push too: release with the tab's Exit (it releases, opens the jaws and exits on the OPEN
report), never with the gripper button followed by a wait.

**2026-10-19 -- the hardware flight version** (record: `archive/box_pushing_hardware_20261019/README.md`).
`AM_HW_PROFILE=box_pushing` on the RAW-mocap whole-body stack (the fused one now refuses it: 0 / 6 fused
pushes in Isaac; the 10-15/16 hardware bags show `/uav_0/mocap` clean). The yaml is the simulation's push
tune + three hardware margins: planner tilt guard 10 deg (below the node's instant 15 deg watchdog), the
arm's fold 2.0 s after a SAFETY revert (the real gripper opens in ~0.7 s) and an abort's 1.5 s hold before
its climb (the jaws open over two WiFi hops). Exit To Push starts by itself 1 s after the push (the arm
GS's `push_pull_auto_exit_delay_s`, 0 = by hand; the sim drivers set it 0 through `ARM_GS_AUTO_EXIT_DELAY`). The planner refuses a frozen or
flipping obj_0 at Get and aborts on a frozen feed in contact; the arm GS takes a full close on the fin as
the grasp on hardware, descends only on the gripper's OPEN report, and never opens without the release.
Rebuild `fsc_trajectory_planner`, `fsc_autopilot_ros2` and `open_manipulator_x_custom_controller` (Orin) and
`utils_custom_ground_station` (laptop). On aarch64 fsc_autopilot_ros2 skips the coordinated-admittance sim fork
(it fetches OSQP; `-DFSC_BUILD_COORDINATED_ADMITTANCE=ON` to build it).
```bash
# laptop: mocap (with the box) + both ground stations, as for pick-and-place
~/source/fsc_autopilot2_ws/src/fsc_optitrack_processor_ros2/scripts/launch_optitrack.sh 192.168.50.230 uav_0 obj_0
# Orin: agent, then the flight stack (RAW mocap) and the torque-mode arm stack
~/dev_ws/src/fsc_autopilot_ros2/scripts/indoor_exp/start_microxrce_agent.sh udp 8888
AM_HW_PROFILE=box_pushing ~/dev_ws/src/fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh uav_0
~/dev_ws/src/fsc_open_manipulator/scripts/indoor_exp/start_open_manipulator_inverted_wb_torque.sh uav_0
```

**2026-10-21 -- the push speed is the one setting** (record: `archive/box_pushing_speed_20261021/README.md`).
The operator types only the Push Distance; the push takes `max(distance / push_pull_push_mean_speed,
push_pull_push_min_time_s)` -- **21 mm/s**, floor 3 s, in the yaml (no value in the planner's code; `push_pull_push_time_s`
is now the valve turn's only). The arm GS's Monitor shows the push time, contact time, mean speed, box travel and box
speed (now vs plan): GREEN inside the tested range, RED outside -- retype (`push_pull_push_distance_ok` 0.10-0.50 m,
`push_pull_push_speed_ok` 8-30 mm/s, yaml, display only); the Push Distance box is amber until sent. 21 mm/s from a 240 g / 0.10 m sweep in Isaac, 12 / 12
complete: 8 mm/s left the box stuck 29 % of the push, 30 mm/s tipped it 0.3-0.7 deg; 0.10 m takes 4.8 s, the jaws
hold it 6.3 s. Rebuild `fsc_trajectory_planner` (Orin) and `utils_custom_ground_station` (laptop); the stack refuses
a planner without the speed.

**2026-10-21 -- the free-axis anchor against the arm's swing** (record: `archive/box_pushing_gain_tune_20261021/README.md`).
The first hardware push ended in a q3-guard abort: with the claw held in the room the arm absorbed the vehicle's drift
and swung, and no gain fixed it. In contact the claw's target now follows 75 % of that drift along the push
(`wb_ee_anchor_free_axis_weight` 0.75, both box-pushing yamls): in Isaac the swing halves and q3 stays >= 19 deg; the
price is the push distance, ±1 cm (worst ±2.5 cm on hardware) whatever the distance. The box-lag guard measures the box
against that shifted target; in Isaac a jammed box was then caught ~1 s later (the stuck box drags the vehicle back and
the target follows), by the 10 deg tilt guard or the box-lag guard, both safely. Ready To Push and Push start by themselves (Go To Start is the one press), Exit 0.2 s
after the push. Rebuild `fsc_trajectory_planner` (Orin) and `utils_custom_ground_station` (laptop); the stack refuses
an older planner.

**2026-10-22 -- the free axis from the claw's room hold** (record: `archive/box_pushing_contact_stability_tune_20261021/README.md`).
The free-axis anchor now also latches during the open-jaw wait on the handle (`wb_ee_anchor_free_axis_on_world: true`,
both box-pushing yamls), not only once the jaws grip -- that wait was where the 10-21 swing started. Tested in a
hardware-like Isaac condition: the real wander comes from PX4's attitude-estimate error (~0.5 deg), emulated as a slow
random plant force (`sim_plant_force_noise_n/_bw_hz/_seed`, 0.32 N / 0.12 Hz); with the anchor off that reproduces the
10-21 abort. On paired seeds 1-8 the old config completed 4/8 with four crashes (the arm buckling in the Ready window);
the new one 6/8 with none (one safe q3-guard abort, one missed grasp). Jams, the side-edge fall and a mocap freeze still
abort safely. No gain changed. Rebuild `fsc_autopilot_ros2` (Orin); the stack
refuses an older node.

**2026-10-22 -- the q3 rate guard** (same record, section 6). The planner now also predicts q3 0.2 s ahead from its own
rate on every arm joint sample, and aborts the push when the prediction falls below 10 deg while q3 falls faster than
5 deg/s for 50 ms (`push_pull_guard_q3_lookahead_s: 0.2`, `_rate_min_deg_s`, `_rate_persist_s`, `_rate_filter_s`; all
three box-pushing yamls; 0 = off, the old check stays). Replayed on the 10-22 runs it fires 0.24-0.35 s before the
static check, at q3 12-21 deg; in the three crashes the static check would have fired only after DIRECT was lost.
The abort it starts is unchanged. Isaac check (4 flights): on the old config's crash seeds 6 and 7 it caught the fold at q3
16-18 deg, the arm stayed above 8.8 deg and the vehicle aborted and landed (last night both left DIRECT at ~33 deg
tilt); no trip on the completed runs. Rebuild `fsc_trajectory_planner` (Orin); both whole-body hardware stacks refuse an
older planner.

### 17.x Comparison rig: coordinated admittance framework (Liu et al., CEP 146 (2024) 105898)

The paper's method as an INDEPENDENT fork, same scene, same box, same driver timetable (2026-10-10;
record `archive/push_pull_coordinated_admittance_20261010/README.md`): flight node
`autopilot_coordinated_admittance_direct_actuation_node` (eqs 11-14 UAV PD, eq 25 momentum wrench
estimator, 250 Hz) + planner node `coordinated_admittance_planner_node` (C++/OSQP: eq 27 admittance on the
EE reference, eqs 31-32 force-adaptive weights, eqs 28-39 kinematic MPC, 20 Hz) + the POSITION-mode arm.
The mission stream (fsc_trajectory_planner) supplies only the EE task; how base and arm share it is the MPC's.
```bash
# 0. clean slate (TWO separate calls), as in 17
# 1. stack (coordinated flight node + its planner node + fsc_trajectory_planner on this fork's mode topic)
WB_SIM_PROFILE=box_pushing ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0
# 2. the scene + PX4 + the POSITION-mode arm stack
PUSH_FEEDBACK=raw ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_box_pushing_sitl.sh shiqi_machine
# 3. as in 17 (ps4_teleop_bringup.py up --hover-z 1.3 --settle 0 --da coordinated_admittance_direct_actuation)
# the whole mission scripted, headless (after step 0); score / summarize:
bash ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_coordinated_admittance_20261010/tools/run_pl_cadm.sh <tag>
PYTHONNOUSERSITE=1 /usr/bin/python3 .../push_pull_coordinated_admittance_20261010/tools/summarize_cadm.py runs/<tag>.npz --metrics
# the results campaign for this method (3 runs) and the two-method MATLAB export / tables
METHOD=coordinated_admittance setsid nohup ~/fsc_PegasusSimulator/results/utils/run_box_pushing_campaign.sh 3 \
    > ~/fsc_PegasusSimulator/results/simulation_results/box_pushing/campaign_cadm.out 2>&1 < /dev/null &
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/fsc_PegasusSimulator/results/utils/build_box_pushing.py
# offline: replay a recorded run's MPC chain / the sway + entry bench (OSQP backend from ~/sources/pyosqp)
PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pyosqp /usr/bin/python3 .../tools/replay_mpc.py runs/<tag>
PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pyosqp /usr/bin/python3 .../tools/sway_bench.py
```
Yamls are GENERATED (`tools/make_coordinated_yaml.py`, from the 4-D whole-body ones: plant and planner
sections verbatim); parity gtests `Coordinated*` (law, estimator, arm model, admittance, weights, the
assembled QP, OSQP vs a 1e-9 reference solve). Design choices the paper leaves open are (a)-(l) in the README.
Results (2026-10-10, 3/3 completed at the first attempt, 525 / 521 / 518 mm; the whole-body rig's 3/3 in
the same table): e_imp slide 2.98 N rms (whole-body 0.66) — 2.94 of it ALONG the push: the admittance
presumes the UAV tracks its base command, this decoupled PD lags it 20-40 mm under the 1 N load, and the
scorer books that as un-rendered impedance; lateral / vertical 0.2-0.4 N. eps_UAV 103 max (the exit) /
29 rms mm (rho 0.279 / 0.079; whole-body 0.100 / 0.047); EE 30.7 mm (18.2); heading 0.96 deg. The
builder aligns the two methods at Ready (this rig's first transit is ~19 s longer: gentler bounds + a
15 s dwell). The driver's live-box grasp gate (`--grip-live-box`, 4 mm across / 25 mm along the fin)
is what this rig needs; the paper-literal gate against the captured box never passed.

## 18. Valve-turning scene (whole-body L1 4-D, contact phase, the push step as a TURN)

**shiqi_machine** -- the box-pushing table (section 17) with the user's VALVE MECHANISM bolted to its
centre (`Valve_Assembly.stl` -> `Valve_Assembly.usda`, 2026-10-15: a 150 x 50 x 20 mm plate, a 608 bearing,
a 225 mm beam on it carrying a 5 x 40 x 130 mm FIN handle at radius 0.160 m, fin top 0.204 m above the
plate; the rotor flown at the printed 204 g, 0.0026 kg m^2 about the axis, on a z REVOLUTE JOINT with a
CONSERVATIVE Coulomb bearing friction 0.048 N m = 0.30 N at the fin, 1.5x the measured <= 0.2 N, no viscous term). The beam points 180 deg (the fin toward the start at
(1.040, 0)). Mocap `obj_0` = the ROTOR: origin on the bearing axis at the plate-bottom plane (= the table
top), yaw = the beam's -- define the hardware rigid body the same way. The mission is the push-and-pull
mode's with **`push_pull_turn_deg` 360**: Go To Start, Ready (open, slide onto the fin), Push = the jaws
close and the claw, the base and the heading ROTATE together about the bearing axis for one revolution
(36 s, arm held, claw world-held, CONTACT on), Exit (open, climb, arm home), Go To Land (270 deg clockwise
to the hover 0.5 m behind the start). Arm pose [0, 5, 25, 0] since 2026-10-18 (was [0, 10, 20, 0]; the same
30 deg fold, README section 12) -- the fin is low: the box push's 60 deg fold would put the T650 skids 2.7 cm
over the beam; here they pass ~14 cm above it, below the fin top, the fin between the skids. `WB_SIM_PROFILE=valve_turning` in step 1 is REQUIRED: step 2 refuses a stack
on another profile (it compares `push_pull_turn_deg` and `push_pull_obstacle` too). Flown 2026-10-15,
six headless runs: at the shipped configuration **2 / 2 full revolutions of the valve, 1 / 2 complete
missions** (v3 complete; v6 turned the beam 363 deg and tripped the 15 deg watchdog at the end of the
turn). The grasp, circle, release and landing chain work; the CONTACT PHASE is marginal -- the base
leads its circle ~2.5 mm per deg/s of yaw rate and the world-held arm absorbs it onto its stops (36 s),
or its ~6 s fore-aft slosh grows past the watchdog (72 s); a CoM-anchored turn wrenched the fin out. A
tuning campaign, not a scene defect. Record + every flight:
`docs/docs_aerial_manipulator/archive/valve_turning_20261015/README.md`. **Tuned 2026-10-17/18** (that README's
section 8): the valve yaml has its own controller tune -- `wb_l1_omega_c_t` 2.0, `wb_ee_anchor_blend_s` 3.0 -- and the
release must open the jaws with the claw still world-held, THEN Exit (the Valve Turning tab's Exit still releases
first, like the box push: use the gripper's Open button, wait ~1 s, then Exit). Results: `results/simulation_results/README.md`.
**Path following (2026-10-18, README section 11):** planner `push_pull_follow` (default false, left off) flies the turn at a governed
progress that slows or holds on the valve's lag, the tilt or the wrist roll. On the measured bearing it never acts (the nominal failure is
the arm's angle-locked fold, not the speed); with a speed-sensitive heavy valve it completed where the scheduled turn hit the force guard.

```bash
# 0. clean slate (TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. stack (4-D L1 node, RAW mocap feedback, emulator with obj_0, planner, drone GS)
WB_SIM_PROFILE=valve_turning ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. the valve scene + PX4 + torque-mode arm stack + arm GS (+ gamepad if plugged in)
VALVE_FEEDBACK=raw ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh shiqi_machine

# 3. take off to 1.3 m, hover 3 s, switch to DIRECT (wait for "Ready for takeoff!" in the PX4 pane first)
FASTRTPS_DEFAULT_PROFILES_FILE=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml \
  /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/ps4_teleop_bringup.py up --hover-z 1.3 --settle 0
```

Or the whole mission scripted, headless (after step 0): `archive/valve_turning_20261015/tools/run_valve.sh <tag>`
(the box push's `pl_mission.py` drives it: the same five services, its "push" step is the turn). Score:
`PYTHONNOUSERSITE=1 /usr/bin/python3 archive/valve_turning_20261015/tools/valve_score.py runs/<tag>.npz [--plot out.png]`
(the beam's rotation per step, the claw's radius about the axis, the base's lead vs the planned yaw rate,
the arm's margin to its stops, the contact truth's tangential force).

**Operating the arm GS "Valve Turning" tab** -- as the Box Pushing tab (section 17), with: **Get** on the Valve row captures the ROTOR
(the axis + the beam's yaw); **EE Offset** [0.160, 0.0006, 0.179] m (the fin's radius and bearing, the claw
25 mm under the fin top; rotor frame); **Turn [°]** 360 (in Push Distance's place; this tab's Plan sends it, the Box Pushing tab's Plan sets 0 -- only
the tab matching the planner's mode is active; the planner refuses a nose off the turn's tangent: `push_pull_yaw_from_box_deg` +90 for a + turn, -90 for -);
**3 Turn** = the jaws close, CONTACT on, 1.5 s settle, the circle. The plan is refused when the gear would
come within 5 cm of the beam top (`push_pull_obstacle`, the valve's swept circle) or the table.

- Scene knobs (env, before step 2): `PEGASUS_VALVE_XY` (1.20,0.00), `PEGASUS_VALVE_YAW_DEG` (180),
  `PEGASUS_VALVE_RISER` (0: a block under the plate -- a higher valve allows a more folded arm),
  `PEGASUS_VALVE_ROTOR_MASS` (0.204; 0 = the asset's solid-PLA 0.243), `PEGASUS_VALVE_DAMPING` (0.0 N m s/rad),
  `PEGASUS_VALVE_FRICTION` (0.048 N m; 0 = the frictionless end), `PEGASUS_VALVE_FIN_GRIP_MM` (20, the sim claw cannot clamp 5 mm),
  `PEGASUS_VALVE_Q2_LIMIT_DEG` (-20,45). The yaml: `archive/valve_turning_20261015/tools/make_valve_turning_yaml.py`
  (the box push's controller tune imported; regenerate, never hand-edit).
- **With the Isaac window (2026-10-18):** step 2 without `PEGASUS_HEADLESS=1` (or `VALVE_WINDOWED=1 run_valve.sh`,
  `tools/chain_win.sh <tag>` for the scripted mission). 06 now draws a frame only while the plant is not behind the
  wall clock (`PEGASUS_RENDER_ADAPTIVE`, default 1; `PEGASUS_RENDER_MAX_GAP_S` 1.0; the Isaac pane's RTF line prints
  fps / frame cost / skipped / longest step / GC) and freezes Python's garbage collector once the scene plays
  (`PEGASUS_GC_FREEZE`, default 1 -- its generation-2 collections were a 170 ms stall every ~10 s with the window and
  the stack up). Measured (win_gc1): free flight RTF 1.000 at 25-31 fps (was 0.81-0.90), no stall > 23 ms anywhere,
  the TURN a smooth 0.92-0.98 (headless 0.98), two windowed missions complete (0 of 8 before). Read the pacer's RTF
  line in the Isaac pane before trusting a windowed contact phase (README 13.3).
- **The ~4 cm in-turn airframe offset (README 13, 15.3):** the law's CoM parks 20-30 mm ahead along the nose =
  `T e_R / k_x` of a standing pitch error that tracks the arm's FOLD (not the turn's speed: a 48 s turn and a third
  of the bearing friction change nothing) -- fold, pitch error, lead, more fold, closed at the stops.
  omega_c_t 2.927 cuts it ~20 % (3 of 3), not shipped. The free-axis anchor (`wb_ee_anchor_free_axis_weight`,
  node option, OFF in the valve yaml) lets the lead turn the valve instead of folding the arm (11 of 11, incl.
  frictionless) but removes the claw's tether on the base (wander x1.5-2, once q1 on its stop and q3 -16 deg);
  never with 2.927. README 15.6 / 15.9; final config 15.10.

**SAFETY FOR THE VALVE (2026-10-18, README 15) -- what changed, and the hardware run:**
- **A failed turn used to be a crash**: the node's 15 deg watchdog tripped first and handed over to SAFETY with
  the jaws still closed on the fin. Now: the planner's tilt guard is **10 deg** (catches 27 of 34 recorded
  failures 0.1-6 s earlier, 0 false trips in the campaign); **the arm GS (both push-and-pull tabs) and the sim
  driver open the jaws when the node leaves DIRECT with the claw on the handle**; the valve's ABORT holds the
  PLANNED pose at the cut, keeps the claw world-held for a 2 s release, then climbs straight
  (`push_pull_abort_release_world_held`, new; injected aborts clean at 20 s frictionless and at 33 s). An abort
  that starts with the arm already on its stops is not recoverable -- the guard is what prevents it.
- **Exit To Turn (arm GS Valve Turning tab) = open the jaws with the claw world-held, Exit 1 s after the OPEN
  report** (`push_pull_valve_exit_wait_s`); the Box Pushing tab keeps its release-first Exit.
- **The FUSED (EKF2) feedback lost the valve 3 of 3 in Isaac** (claw ~1 cm off in contact: the jaws closed on
  nothing once, the fin lost twice, crashes). The fused hardware stack REFUSES `AM_HW_PROFILE=valve_turning`
  (`ALLOW_FUSED_CONTACT=1` overrides, A/B only) and warns for box_pushing. **Fly the valve on the RAW stack.**
- **The bearing's friction is a safety parameter**: the shipped turn completes from 0.005 N.m up and only 1 of 3
  frictionless. The real 608RS is <= 32 mN.m and "probably a few": measure its breakaway torque before the first
  turn (a weight on a string at the fin, r = 0.160 m: 3 g = 5 mN.m); below ~5 mN.m add friction to the shaft.
- **Hardware run** (Orin: rebuild `fsc_trajectory_planner` and `fsc_autopilot_ros2`; laptop: rebuild
  `utils_custom_ground_station`; MEASURE `push_pull_table` / `push_pull_obstacle` into
  `archive/valve_turning_20261015/tools/make_hw_valve_turning_yaml.py` and regenerate):
```bash
# fsc_autopilot_ros2/scripts/indoor_exp/ -- the RAW-mocap whole-body stack, not the _fused one
AM_HW_PROFILE=valve_turning ./start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh
```
