# Task: PX4-side analysis of the 2026-10-19 PX4 <-> Orin link outage (ground-station laptop)

ULOG_DIR = <FILL IN: the folder you exported the .ulg files to from QGC>
OUT      = ~/eth_link_test_20261020/laptop        (create it; if the test day differs, use that date
                                                    here AND in the Orin prompt)

You are on the ground-station laptop (it runs QGroundControl, the mocap processor, both ground stations
and the ROS 2 bag recorder). Work read-only towards the vehicle: no PX4 parameter changes, no reboots,
do not touch running ground stations, no sudo. An operator is with you.

## Background (settled from the ROS bags: do not re-analyse)
On 2026-10-19, during a whole-body pick-and-drop flight, all uXRCE-DDS traffic between PX4 (Pixhawk 6X)
and the MicroXRCE Agent on the Jetson Orin stopped in BOTH directions, twice: 2.4 s, then 18.0 s. The
link is Ethernet (hand-made JST-GH 4-pin -> RJ45 cable, Orin 192.168.0.1 <-> PX4 192.168.0.3, UDP 8888).
WiFi between the Orin and this laptop carried every other ROS topic at full rate the whole time, no Orin
core was saturated, and PX4 did not reboot. The flight node fell back to SAFETY; PX4, with neither
offboard commands nor mocap, flew away, dropped and flipped. The same signature happened before on
2026-10-05 16:40:07 EDT (on the ground), 2026-10-09 18:04:53 EDT (in flight) and 2026-10-09
18:19:47 EDT (on the ground after disarm).
Still open: a physical contact fault (cable/connector), an Agent stall, or a PX4-side stall. The Orin's
kernel log and a bench test (a later session, on the Orin) cover the first two. YOUR job is PX4's side:
what PX4 heard from the Orin and when, whether PX4 itself was healthy at the cut, and what it did alone.
You also check this laptop's WiFi history.

## Time mapping (from the bag's /uav_0/fmu/out/timesync_status, +-5 ms)
ULog timestamps are microseconds since PX4 boot. **PX4 boot time [s] = epoch - 1792443851.351**
(- 1792443851.356 after the 18 s outage). Both 10-19 flights are in ONE Pixhawk boot (powered on
~17:04:11 EDT), so the pick-and-drop log covers boot time ~270-344 s and the hover log ~950-1040 s.
Pick-and-drop anchors (bag time t -> PX4 boot s = t + 278.224):

| event | EDT | PX4 boot s |
|---|---|---|
| OFFBOARD entered | 17:09:00.16 | 288.81 |
| last PX4->Orin message before cut 1 | 17:09:34.459 | 323.108 |
| PX4 messages back | 17:09:36.834 | 325.483 |
| PX4 reports nav POSCTL + failsafe | | 325.507 |
| PX4 back to OFFBOARD | | 325.517 |
| last PX4->Orin message before cut 2 | 17:09:37.329 | 325.978 |
| vehicle starts dropping from 1.5 m | | ~334.3 |
| flip | | ~335.4-335.7 |
| righted by hand | | ~340.5-343.2 |
| PX4 messages back, already disarmed, STAB | 17:09:55.318 | 343.962 (status stamped 343.84) |

Hover log (control, no outage): OFFBOARD ~969.6 s, disarm ~1037.9 s.

## Steps
0. Tools: check `python3 -c "import pyulog"`; else `pip install --user pyulog` (needs internet). If the
   laptop is offline, write a minimal ULog reader from the ULog spec (docs.px4.io, "ULog File Format").
   Save every extracted series as CSV in $OUT/csv/.
1. Inventory every .ulg in ULOG_DIR: time range (boot s), duration, arming periods, firmware/sys info,
   logger dropouts, number of logged messages. Identify the pick-and-drop log and the hover log; list
   any 10-05 / 10-09 logs too.
2. Pick-and-drop log, with numbers (PX4 boot s and EDT):
   - Q1 Orin -> PX4. For every topic PX4 receives from the Orin that is in the log (vehicle_visual_odometry
     or vehicle_mocap_odometry, offboard_control_mode, actuator_motors, vehicle_thrust_setpoint,
     vehicle_torque_setpoint, vehicle_attitude_setpoint, timesync_status if logged): the last sample
     before 323.108 and the first after, and the same around 325.978 and 343.96. State each topic's
     logged rate, so the precision is clear. Did PX4 stop hearing the Orin at the same instant as the
     Orin stopped hearing PX4? Which direction stopped first?
   - Q2 Was PX4 itself healthy over 315-345 s? Logger dropouts (time and length); gaps in high-rate
     internal topics (sensor_combined, vehicle_attitude, vehicle_angular_velocity); cpuload (load, RAM);
     EVERY logged message from 300-350 s, all levels, and anything anywhere mentioning uxrce, dds, net,
     eth, timesync, offboard or failsafe.
   - Q3 Power: battery_status voltage and current at 320-326 s (a dip or spike at 323.1 s?), and
     system_power (5 V rail, brick status) if logged.
   - Q4 What PX4 did alone (325.98-343.96 s): the nav_state / failsafe / failsafe_flags sequence,
     EV fusion and local-position validity, pilot input (manual_control_setpoint, input_rc: mode
     switch, throttle, kill switch), actuator_outputs, the drop and the flip, and when and how it
     disarmed (vehicle_command, logged messages, failure detector).
3. Control and history: Q1-Q2 for the hover log (expect clean). For any 10-05 / 10-09 logs present,
   Q1-Q2 around their events. If those days' bags are on this laptop, take that boot's offset from their
   /uav_0/fmu/out/timesync_status estimated_offset.
4. This laptop's WiFi on 10-19 (the operator lost WiFi/SSH to the Orin several times that afternoon,
   even with the vehicle on the ground; the repeater was NOT connected to its usual phone-hotspot uplink
   that day):
   - `journalctl -u NetworkManager -u wpa_supplicant --since "2026-10-19 12:00" --until "2026-10-20 00:00"`:
     disconnects, reason codes, BSSID/channel changes, DHCP, connectivity state.
   - `nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY con show`: could another known network pull the
     laptop away?
   - `iw dev <wifi> get power_save`, `ip route`, `nmcli -f all dev wifi list` (the repeater's channel,
     other APs on it).
5. Optional WiFi A/B, only if the operator agrees and the Orin is powered and on WiFi: ping the Orin
   (Lab 192.168.50.107 / Dome 192.168.2.107) with `ping -D -O -i 0.2` for 10 min with the phone hotspot
   connected to the repeater and 10 min without, logging `iw dev <wifi> link` once a second, and writing
   a marker line when the operator toggles. Report loss and the longest outage for each condition.
6. Write $OUT/REPORT_laptop.md:
   - the log inventory;
   - a Q1 table (stream, last sample before / first after, delta to the PX4->Orin cut);
   - Q2-Q4 with times (PX4 boot s and EDT);
   - one figure for 320-345 s if matplotlib works (Orin->PX4 stream presence, PX4 internal topics,
     cpuload, battery current, nav_state);
   - the WiFi findings;
   - a conclusion from the PX4 side: (a) PX4 healthy and both directions cut at the same instant
     (cable or Agent; the Orin decides); (b) a PX4-side stall (dropouts, internal gaps, CPU); or
     (c) not decisive.
   Raw CSVs stay in $OUT/csv/.
7. Copy everything to the Orin for the next session (ask the operator for the Orin's IP):
   `ssh jetson@<ip> mkdir -p ~/eth_link_test_20261020 && scp -r ~/eth_link_test_20261020/laptop jetson@<ip>:~/eth_link_test_20261020/`
   Also copy the .ulg files you used into $OUT/px4_logs/ before the scp.
