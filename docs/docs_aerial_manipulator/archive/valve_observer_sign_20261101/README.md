# Does the observer read the valve's tangential force with the wrong sign in Isaac? (2026-11-01)

User question, after a review of the valve-turning bags reported that during valve slips the tangential force
estimates swing 1.4–2.6 N RMS and average in the wrong direction or near zero (current −1.5 to −2.4 N, command
−0.7 to −0.9 N, observer −1.1 to +0.2 N). That review's script and sign convention are not in the repo, so
everything here uses one stated convention.

## Convention

- **The force ON THE CLAW**, resolved on the valve's tangent at the fin.
- A valve resisting its own motion pushes the claw backward, so **the correct sign is NEGATIVE**.
- Expected size: the bearing alone is 0.072 N·m / 0.160 m = 0.45 N, plus the valve's inertia.
- **Slips** = the valve turning faster than 15 deg/s, either way; scored along the valve's own motion.

Quantities:
- **truth**: minus the scene's PhysX contact force of the vehicle on the valve (`contact` [1:7]). The sign is
  checked per run: the vehicle's mean power on the rotor is positive in all 23 runs.
- **obs**: the observer's raw reading (`wb_control_debug` [97..99]). It solves J_yq^T F = the joint-row
  residual, which makes it the force on the claw, in world axes.
- **used**: what the law consumed (`wb_control_debug` [58..60]) = the raw reading through C_x (omega_x 2.0).

## Isaac (23 valve turns: the v2 valve at 0.072 N·m, drive model, 90 deg)

Runs: `two_param_sets_20261101` ts_*, `valve_room_hold_guards_20261031` rh_*, `valve_safety_fixes_20261030`
vs_*. Tool: `tools/obs_sign.py`.

- **Over the whole turn the raw reading is right.** Its mean equals the truth's to within ~0.15 N in every run
  (truth −0.04 to −1.41 N).
- **During slips the raw reading is pulled toward zero.**
  - Truth along the motion: −0.39 to −1.47 N, negative in all 23.
  - Raw observer: −1.12 to +0.18 N. It keeps 3–102 % of the truth, and 3 runs read the WRONG sign
    (rh_nom2 +0.18, vs_nom_2 +0.10, vs_nom_3 +0.14).
  - Its sd is 2.0–2.9 N, about the size the review saw on hardware.
- **The law's filtered reading has the right sign in all 23**: −0.20 to −1.26 N, mostly 60–100 % of the truth.
- **The error is not a scaled-down truth** (`tools/obs_error.py`, error low-passed at 1.6 Hz):
  - the truth alone explains R² ≤ 0.04 of it;
  - adding the arm joints' motion (friction shape tanh(qdot/0.015) and joint acceleration) raises that to
    0.20–0.53;
  - joint motion alone accounts for about half to two thirds of the slip-window bias;
  - the CoM's and the valve's accelerations explain nothing.
- **Reading:** in contact the 4-D attribution freezes the joint-row trim ŵ_q. Any joint-row disturbance that
  changes while the arm moves (friction reversing, inertia mismatch) is then booked as task force, and it opposes
  the contact reading during a slip. **Inferred from the regression, not isolated by a flight.**

## Hardware, same definitions (`tools/obs_sign_hw.py`)

No truth on hardware. The tangent comes from the circle fitted to the streamed r_ed; the valve's rate from
obj_0's yaw.

| flight | valve turned | raw obs, slips along motion | used, slips along motion | raw sd |
|---|---|---|---|---|
| 10-28 180652 | +25 deg | −0.10 N | −0.02 N | 5.2 N |
| 10-28 181833 | +7 deg | −0.15 N | +0.04 N | 6.7 N |
| 10-29 145526 | −27 deg (29 % of samples slipping backward) | **+1.26 N** | **+0.45 N** | 2.8 N |

- On hardware the filtered reading is also near zero or wrong-signed, where in Isaac it was right.
- Hardware arm friction is larger and stick-slips, which would fit a larger version of the same joint-row
  error. **Hypothesis, not shown.**

**Open:**
- the review's sign convention: negative is right under this one, so its "current" and "command" values would
  then be right-signed;
- which joint carries the error;
- the "current" and "command" estimates were not rebuilt here.
