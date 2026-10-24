# Task: find out why the PX4 <-> Orin Ethernet link drops (Orin side + bench test, props OFF)

D = ~/eth_link_test_20261020     (the laptop session already copied its results into $D/laptop/;
                                  if the test day differs, use the same date as the laptop prompt)

You are running on the Jetson Orin of the AM-T650 aerial manipulator (user `jetson`). An operator is at
the vehicle, and the ground-station laptop (QGroundControl, ground stations) is on the same WiFi. You
cannot run `sudo` (there is no terminal for the password): hand any sudo command to the operator, with
its output tee'd into $D.

## Background (settled from the ROS bags: do not re-analyse)
On 2026-10-19, during a pick-and-drop flight, all uXRCE-DDS traffic between PX4 (Pixhawk 6X) and the
MicroXRCE Agent on this Orin stopped in BOTH directions:
- 17:09:34.46 -> 17:09:36.84 EDT (2.4 s), then 17:09:37.33 -> 17:09:55.32 EDT (18.0 s)
  = 21:09:34-21:09:55 UTC = epoch 1792444174.5-1792444195.3.
- WiFi to the laptop carried every other ROS topic at full rate the whole time, no Orin core was
  saturated, and PX4 did not reboot. The flight node fell back to SAFETY; PX4, with neither offboard
  commands nor mocap, flew away and flipped.
- The same signature (PX4 topics gone, the Orin's own topics fine) happened before:
  2026-10-05 16:40:07 EDT (on the ground, just landed), 2026-10-09 18:04:53 EDT (in flight, 5.2 s),
  2026-10-09 18:19:47 EDT (on the ground, after disarm). There were none in 9 flights 10-10 ... 10-16.
- While the link is up it is clean (no scattered frame loss), so it is not a noisy signal.
- Open question: an intermittent PHYSICAL contact (hand-made cable: Pixhawk JST-GH 4-pin -> RJ45, two
  twisted pairs), an Agent stall, or a PX4-side stall.
**Start by reading $D/laptop/REPORT_laptop.md** (the PX4 flight-log analysis and the laptop's WiFi
history). Summarize its conclusion to the operator and use it below. If it is missing, say so and go on.
Your job: the Orin's evidence, a guided bench test, and the combined verdict.

## Facts to verify (do not assume them)
- Ethernet to PX4: interface `enP8p1s0`, Orin 192.168.0.1/24, PX4 192.168.0.3. Agent:
  `MicroXRCEAgent udp4 -p 8888` in tmux session `fsc_microxrce_agent`. PX4 topics under /uav_0.
- WiFi: find the interface (`nmcli dev`), the SSID and the Orin's WiFi IP; ask the operator for the
  laptop's IP.
- ROS: `source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash`. CPU map
  (~/dev_ws/src/fsc_autopilot_ros2/scripts/realtime/cpu_layout.sh): CPUs 0-1 OS/IRQs, 0-2 estimator,
  2 planner, 3 arm real-time thread, 4 Agent, 5 flight node.
- Clock: `timedatectl` (timezone, NTP). Journal persistence: does /var/log/journal exist?
  `journalctl --list-boots`.

## Rules
- Props OFF the whole time. Never switch the flight node to DIRECT.
- Do NOT start, stop or restart the Agent, the flight stack, the arm stack or PX4 (a restarted Agent may
  not reconnect to PX4 without a Pixhawk reboot). Do not change PX4 parameters. Do not edit or git-pull
  any repo. Do not touch scripts/realtime or the CPU isolation.
- Run everything you start with `taskset -c 0-1`, at normal priority and low rates; never on CPUs 3-5.
- All output goes into $D (never inside $D/laptop/). Never delete anything. Timestamp every line with
  epoch seconds.
- Ask before doing anything that is not in this plan.

## Phase 0: evidence that a reboot or rework destroys (do this first, ~5 min) -> $D/phase0/
1. `timedatectl`, `uptime`, `journalctl --list-boots`, `last -x | head -40`, `ip -br addr`, `ip route`,
   `nmcli dev`, `nmcli -f all con show --active`.
2. Kernel log around each of the four events (+-2 min, every boot the journal still has):
   `journalctl -k -o short-precise --since ... --until ...`, plus a full grep of each boot's kernel log for
   `enP8p1s0|r816|link|NETDEV|timed out|reset`. If the 10-19 boot is not in the journal, say so.
3. WiFi on 2026-10-19 (the operator lost WiFi/SSH several times that afternoon, even on the ground; the
   repeater had no phone-hotspot uplink that day):
   `journalctl -u NetworkManager -u wpa_supplicant --since "2026-10-19 12:00" --until "2026-10-20 00:00"`,
   grep `disconnect|deauth|reason|beacon|CTRL-EVENT|DHCP|lease`. List each drop with its time, reason
   code and who initiated it, and compare with the laptop's list in REPORT_laptop.md (the same
   moments = the network; only one side = that machine).
4. Why the Agent and the flight stack were sometimes gone after the operator reconnected SSH: boots that
   day, `grep -i KillUserProcesses /etc/systemd/logind.conf /etc/systemd/logind.conf.d/* 2>/dev/null`,
   `loginctl show-user jetson`, `journalctl -k | grep -i "out of memory"`.
5. Agent history: `tmux ls`, then
   `tmux capture-pane -p -J -S - -t fsc_microxrce_agent > $D/phase0/agent_scrollback.txt`.
6. Interface counters now: `cat /sys/class/net/enP8p1s0/{carrier,speed,duplex,carrier_changes,carrier_up_count,carrier_down_count}`,
   `ip -s -s link show enP8p1s0`, and `ethtool enP8p1s0` / `ethtool -S enP8p1s0` if they run without root.
Summarize Phase 0 for the operator before going on. Above all: was there a link-down at 21:09:34 UTC
on 10-19?

## Phase 1: start the loggers (one detached tmux session `eth_test`, one window each)
1. Link monitor. Write `$D/eth_link_monitor.py` (rclpy + px4_msgs, best-effort QoS). Subscribe to
   /uav_0/fmu/out/vehicle_attitude (expect ~400 Hz), /uav_0/fmu/out/timesync_status and
   /uav_0/fmu/out/vehicle_status_v1. Every 0.5 s append a CSV row: epoch, attitude messages in the
   window, largest inter-arrival gap [ms], age of the last attitude message [s], arming/nav state,
   carrier, carrier_changes, rx/tx packets, rx_errors, rx_crc_errors, rx_dropped, rx_missed_errors,
   tx_errors (from /sys/class/net/enP8p1s0). Write an EVENT line when the attitude stream is silent for
   more than 100 ms and when it resumes (with the duration), and whenever carrier or carrier_changes
   changes. Print a one-line live status.
2. Kernel follow: `journalctl -k -f -o short-unix > $D/kernel_follow.log`. If refused, hand
   `sudo dmesg -w --time-format iso | tee $D/kernel_follow.log` to the operator.
3. Ping PX4 over Ethernet: `ping -D -O -i 0.2 192.168.0.3 > $D/ping_px4.log`. Check that PX4 answers at
   baseline; if it never answers ICMP, note it and stop this logger.
4. WiFi: `ping -D -O -i 0.2 <laptop IP> > $D/ping_laptop.log`, plus the `iw dev <wifi> link` output
   (signal, bitrate) once a second with an epoch prefix.
5. CPU per core once a second with an epoch (tegrastats `--interval 1000 --logfile`, or a /proc/stat
   sampler), plus /uav_0/system_monitor/status if that topic exists.
6. The Agent pane from now on: `tmux pipe-pane -o -t fsc_microxrce_agent "cat >> $D/agent_pane.log"`
   (check first that it is not already piped).
7. A small Orin-side bag (check free disk first): `taskset -c 0-1 ros2 bag record -o $D/bag_orin`
   /uav_0/fmu/out/vehicle_attitude /uav_0/fmu/out/timesync_status /uav_0/fmu/out/vehicle_status_v1
   /uav_0/fmu/out/estimator_status_flags /uav_0/fmu/out/battery_status
   /uav_0/fmu/in/offboard_control_mode /uav_0/fmu/in/vehicle_visual_odometry
   /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/mode /uav_0/system_monitor/status /rosout
Confirm that every logger is writing before Phase 2.

## Phase 2: guided bench steps (the operator does the physical part)
When the operator says "start Sx" / "stop", append `<epoch> Sx start|stop <note>` to $D/markers.txt.
After each step report: link EVENTs, the carrier_changes delta, error-counter deltas, ping losses (PX4
and laptop), the largest PX4 gap.
- S0 baseline: untouched for 3 min (stacks up, SAFETY, disarmed).
- S1 Pixhawk end: wiggle, gently tug and press the JST-GH plug and the wires right behind it (1-2 min).
- S2 Orin end: the same at the RJ45 plug.
- S3 the run: flex and pinch along the cable and move each pair separately, especially near the arm,
  the power leads, and any point that moves with the arm.
- S4 the arm: drive it through its range from the arm ground station (SAFETY targets: home, the pick
  pose [0, 32, 38, 0] deg, the extremes) and open/close the gripper several times. Then hang the ~190 g
  basket on the gripper and let the arm take its weight suddenly (on 10-19 the link cut exactly as the
  basket's weight came on the arm).
- S5 the frame: tap and shake the airframe, lift it and set it down on its skids.
- S6 motors, props OFF: the operator arms with the RC in STAB (never DIRECT), idles 30 s, then raises
  the throttle in steps up to high, 30 s each, then disarms.
- S7 soak: everything untouched for 20-30 min.
If the PX4 stream stops at any point: immediately save `ip -s -s link show enP8p1s0`, the sysfs counters,
and the last 200 lines of the Agent pane and of kernel_follow.log into $D/event_<epoch>/. Ask the
operator to run `uxrce_dds_client status` and `netman show` in the QGC MAVLink console (laptop) and paste
the output to you. Do not restart anything; note whether and when the link recovers by itself. If it
happened while armed (S6), tell the operator to export that flight log from QGC afterwards.

## Phase 3: combined verdict -> $D/REPORT.md
Include: the environment, the laptop report's PX4-side conclusion, the Phase 0 findings, a per-step table
(EVENTs, carrier_changes, error deltas, ping loss to PX4 and to the laptop, largest PX4 gap, max CPU per
core), and the verdict:
- kernel "Link is Down", carrier_changes rising, or PX4 ping lost together with the stream
  -> physical link (cable or connector): name the step and the end.
- error counters climbing while the link stays up -> signal integrity (pairing, twist, routing near power).
- stream silent, carrier up, no errors, PX4 still answering ping -> the uXRCE session layer: use the
  Agent log (session delete/create lines), `uxrce_dds_client status` and the PX4 log findings to say
  whether the Agent or PX4's client stopped.
- stream silent, carrier up, PX4 ping also lost -> PX4's network stack or the Orin's NIC driver (look for
  NETDEV watchdog / transmit timeout lines).
- nothing reproduced -> inconclusive. Say so, and recommend running the link monitor and the kernel
  follow during every flight until it happens again.
Also answer: the reasons for the 10-19 WiFi drops (Orin and laptop side together), and why the
Agent/stack were gone after SSH reconnects.

## Phase 4: wrap up
Stop only what you started (the `eth_test` session; `tmux pipe-pane -t fsc_microxrce_agent` with no
command stops the piping). List $D with file sizes, and give the operator the scp command to copy $D
back to the laptop.
