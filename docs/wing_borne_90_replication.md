# Replicating 90° wing-borne cruise — step by step

Flight-validated on 2026-09-07. Reference bags: `full_tr30` (228 s of steady cruise),
`full_tr32` (back-transition with aft rotor tilt, 2.2 m altitude excursion).

**What you should get:** hover → rolling transition → tilt 90°, airspeed ~15.9 m/s, rotors at
4.7 N of a 33.8 N weight (drag only — the wing carries 101%), altitude and airspeed constant to
0.01 m and 0.01 m/s, for as long as you leave it there. Then a back-transition that stops the
aircraft with a ~2 m altitude excursion.

---

## 0. Before every test — restart the whole stack

```bash
bash scripts/restart_hil.sh
```

**Do not skip this.** Four flights in the campaign were lost to stale-stack faults, none of which
announced itself: PX4's sim battery hitting its floor and firing a Land failsafe while the
controller commanded correct thrust; the HIL link dying silently (position frozen to four decimals
while commands varied); a 34 m/s runaway on a reference retained across a restart; and the aircraft
3.8 km from origin where the safe controller's envelope refused to recover it.

The script kills the nodes → PX4 → Isaac, starts **Isaac first** (it waits for the heartbeat) then
PX4, re-applies the PX4 params that do not survive a restart, brings both controllers back, and
**refuses to report ready** unless there are zero poll timeouts, Isaac is receiving HIL frames and
the NMPC node has no traceback. Wait for `READY`.

---

## 1. From the ground station — the whole flight, no terminal

The Trajectory tab now has everything: a **Vehicle** row (arm / offboard / takeoff / land) and the
wing-borne pair. A complete sortie is six clicks.

| # | click | what happens |
|---|---|---|
| 1 | **Safe Geometric** | engages the safe hover controller — it must be active before arming, because PX4 only accepts OFFBOARD while a controller is streaming `OffboardControlMode` |
| 2 | set **Z** in the position boxes, press **▲ TAKEOFF** | requests OFFBOARD, arms, and climbs straight up from where the vehicle is |
| 3 | **✈ Wing-borne 90° (validated)** | applies the flight-validated configuration and engages the full-state NMPC. **Refuses below 60 m** — the transition spends ~25 m buying flying speed, and starting too low aborts it at the moment it succeeds (`full_tr19`) |
| 4 | give it a far target (fly-to-point), or leave it | accelerates on a gentle ramp, descends ~20 m while under-speed, levels off past 14.5 m/s, settles at tilt 90° |
| 5 | **🛬 Back-transition → hover**, or just **STOP** | decelerates on the trim schedule (nose-up flare + aft rotor tilt), hands to Safe Hover automatically below 3 m/s, times out after 60 s so `decel_now` can never stay latched |
| 6 | **▼ LAND** | descends at the current X/Y and disarms once settled |

**STOP is safe from cruise now.** Pressing STOP above 5 m/s on the full-state NMPC runs the
back-transition instead of handing over immediately. That handover is what destroyed bag
`full_tr30`: tilt snapped to zero at 16 m/s, lift doubled, it stalled at AoA +78° and departed with
a 65.7 m excursion.

**The back-transition also arms itself.** The reference watches the distance to the commanded
destination and starts decelerating once it is inside `V²/(2·decel_rate) + 20 m`, so a mission that
flies to a point slows down on the schedule by itself — you do not have to remember to press
anything.

**DISARM refuses while airborne and moving**, and LAND disarms only once it is below 0.25 m and
slow.

## 2. From the command line (what the campaign actually flew)

```bash
bash scripts/restart_hil.sh          # wait for READY

source scripts/rosenv.sh
cd /tmp/bags && ros2 bag record -o myrun \
  /uav_0/mocap /uav_0/fmu/in/actuator_motors /uav_0/fmu/in/actuator_servos \
  /uav_0/ground_station/active_controller &

cd /home/pieper/envs/fsc_PegasusSimulator
python3 scripts/fly_transition.py --controller nmpc_full \
    --alt 110 --dist 2500 --climb-time 100 --cruise-time 150 \
    --back-transition --bt-time 70 --recover-time 45
```

`fly_transition.py` sets nothing itself — set the parameters first (or press the GS button):

```bash
for kv in "ttr 40.0" "trans_sink 0.8" "v_lead 2.5" "vcruise 16.0" "wingborne_v 14.5" \
          "descent_budget 45.0" "cruise_alt_band 60.0" "tilt_margin_deg 8.0" \
          "decel_rate 1.5" "flare_deg 15.0" "beta_aft_deg 20.0"; do
  ros2 param set /nmpc_full_smc_control ${kv% *} ${kv#* }
done
ros2 param set /geometric_hover_control z_max 130.0
ros2 param set /geometric_hover_control radius_max 8000.0
ros2 param set /geometric_hover_control radius_local 2000.0
```

## 3. Reading the result

```bash
python3 scripts/ana_bag6.py myrun          # or the episode analyser
```

A healthy run reads: **tilt 90.0, AoA ≈ +6, wing share ≈ 100%, rotor thrust ≈ 5 N, |vz| ≈ 0**.

`fly_transition.py` prints the verdict itself. Trust only these two lines:

- `longest UNBROKEN controlled+level run` — consecutive samples that are level **and** flown
- `fastest CONTROLLED sample`

**Ignore peak speed and peak wing share.** They lie, and did so three times in this campaign: a
dive scored 15.8 m/s, a post-dropout glide scored 80% wing share, and a phugoid's zero-crossings
scored "TRUE FIXED-WING". A glide after the controller falls back is level, fast *and* wing-loaded.
The verdict now requires rotor thrust < 32 N (i.e. not the hover fallback) and consecutive samples.

---

## Why each parameter is what it is

| parameter | value | why (which bag proved it) |
|---|---|---|
| `ttr` | 40 s | The phugoid is a **decaying** ~10 s mode (measured −0.084 m/s), not a limit cycle. An aggressive ramp rings it and the aircraft porpoises 63–90 m for minutes (`full_tr20`). Gentling it gave 228 s of steady cruise (`full_tr30`). |
| `trans_sink` | 0.8 m/s | Below the 14.13 m/s stall the wing cannot carry the weight, so a *level* reference forces the solver to buy lift with angle of attack — it mushed at AoA +28…+35° on the back side of the drag curve (`full_tr18`). The descent must be **commanded**; merely permitting it changes nothing, because the cost still demands the old altitude. |
| `descent_budget` | 45 m | The dive-out needs ~25 m. Too small and the safety band terminates the manoeuvre exactly as it succeeds (`full_tr19`, dumped at 24.8 m while at 18.8 m/s and 90° tilt). |
| `flare_deg` | 15° | Nose-up braking, and it is what keeps the deceleration **controlled** — not just extra braking. At 5° the aft tilt still brakes hard and the aircraft then departs: AoA to 172° (flying backwards), 40 m lost (`full_tr35`). With `β ≥ 0` pitch is the *only* decelerating force, and it converts speed into height, so it alone cannot beat a 12.8 m balloon. |
| `beta_aft_deg` | 20° | Front rotors lean **aft**, putting braking force along the flight path so the body stays closer to level. Excursion 18.9 m → **2.2 m** (`full_tr32`), below the 12.8 m pitch-only floor. It is an **assist, not a replacement**: with the flare cut to 5° it brakes but the aircraft departs (`full_tr35`). |
| `tilt_margin_deg` | 8° | Applied tilt ≤ trim tilt(V) + margin. Over-tilting at low airspeed leaves no pitch authority (front rotors horizontal, elevator with no dynamic pressure) and the nose simply falls. |

## Airframe change this depends on

`beta_aft_deg` requires the front rotors to travel **past vertical**: `tilt_min_deg = −20` in
`quad_tiltrotor_config()`. If the real mechanism cannot do that, set `beta_aft_deg = 0` and
`flare_deg = 25` — the flare-only back-transition still works, with an 18.9 m excursion.
**Never lower `flare_deg` below ~15 while aft tilt is enabled** — the pair are complementary and
the aircraft departs without enough nose-up (`full_tr35`).

The sim's tilt mapping is anchored at zero (`_tilt_target_rad` scales against `tilt_max` and clips
to `[tilt_min, tilt_max]`), so **command 0 always means 0° / hover**. Do not change it back to
`lo + (hi−lo)·cmd`: with a negative `tilt_min` that makes hover's zero mean −20°, and the safe
controller writes literal zeros on the tilt channels. This project has already had one aircraft
tilt on arm from exactly that mistake.
