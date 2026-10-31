#!/usr/bin/env python3
"""Reference implementation of the raw-mocap safety guard's sample checks.

Pure numpy: this is the specification the C++ estimator guard must reproduce
(the repo's Python-reference -> C++ -> parity pattern). It classifies every
mocap sample of the drone body as GOOD or BAD and runs the agreed time ladder
on the age of the last GOOD sample:

    age > T_BRIDGE  (or the latest sample is BAD)  -> BRIDGING  (EKF2 + frozen offset)
    age > T_CONFIRM                                 -> CONFIRMED (in contact: abort)
    age > T_LOST                                    -> LOST      (node failsafe)

A sample is BAD when it is
  * FROZEN : its position is bit-identical to the previous sample's and the
             pose has not changed for > FROZEN_S (>= 3 identical samples --
             repeated VRPN frames come in pairs, 2026-10-14);
  * a JUMP : farther than the gate from the LAST GOOD sample's position (so
             the true sample after a one-sample flip is accepted again), or
             rotated more than the gate from it. Both gates grow with the time
             since the last good sample: the position gate by a bounded speed
             GATE_V and acceleration GATE_A, the attitude gate by GATE_W_DEG.
             NO velocity prediction, on purpose, after two failures replaying
             the recorded flights: the processor's twist is a one-sample finite
             difference that spikes to tens of m/s across a flip or a frame
             skip (one rejection then lasted > 20 s); the guard's own finite
             difference is corrupted by any outlier just inside the gate (a
             20 mm one-sample glitch on 09-21 became a false confirmation).
             The flips are 110-190 mm in one sample; a normal 8 ms step is
             < 6 mm, so a speed-bounded gate separates them without velocity.
             PREDICTED CENTRE (2026-10-30, the 10-29 valve crash): a body moving
             faster than GATE_V leaves that band and can never get back in -- at
             2.8 m/s the true samples were rejected from 46.99 s, LOST at 48.0,
             and EKF2 then flew blind into the room's edge. So the position gate
             ALSO accepts a sample near the last good position moved on by
             EKF2's velocity (PX4's IMU estimate -- not the processor's twist,
             not the guard's own difference: neither failure above applies),
             averaged over the gap (last good sample's and now's) and capped at
             PRED_V_MAX -- and only when that speed exceeds PRED_V_MIN (= GATE_V:
             below it the band's own growth covers the motion), so every verdict
             in slower flight is the 10-21 guard's exactly (replayed: 3 borderline
             29 mm one-sample jumps at 0.25 m/s would otherwise turn good). It is
             a UNION: a sample inside the old band is still good, so a wrong EKF2
             velocity can only ever fail to help -- it can never reject a sample
             the 10-21 guard accepted. A flip (110-190 mm) stays outside both
             centres below ~12 m/s; the attitude gate is unchanged.

A stopped stream is caught by the age alone. After a CONFIRMED episode the
feed must stay good for T_RECOVER before the guard returns to MOCAP; a shorter
episode (a dropped sample, one rejected flip) returns at the next good sample.
"""

from dataclasses import dataclass, field

import numpy as np

# ---- the agreed thresholds (2026-10-21) -------------------------------------
T_BRIDGE = 0.050     # [s] normal gaps reach 31 ms (10-15/16 flights)
T_CONFIRM = 0.150    # [s] 5x the worst normal gap
T_LOST = 1.0         # [s] = the flight node's system_fb_timeout_s
T_RECOVER = 1.0      # [s] good feed needed after a CONFIRMED episode
FROZEN_S = 0.050     # [s]
FROZEN_N = 3         # samples
JUMP_P = 0.020       # [m]   normal worst 5.0 mm
JUMP_R_DEG = 10.0    # [deg] normal worst 1.9 deg
GATE_V = 1.0         # [m/s] speed the position gate allows per second of gap (task speeds <= 0.6)
GATE_A = 2.0         # [m/s^2] acceleration the position gate allows over a gap
GATE_W_DEG = 90.0    # [deg/s] rotation rate the attitude gate allows over a gap
PRED_V_MIN = 1.0     # [m/s] the predicted centre is used only above this (= GATE_V)
PRED_V_MAX = 4.0     # [m/s] cap on the EKF2 velocity that moves the predicted centre
EKF_STALE_S = 0.1    # [s] an older EKF2 velocity is not used (= the bridge's ekf_stale_s)


def quat_angle_deg(q1, q2):
    """Angle between two unit quaternions [w, x, y, z] in degrees."""
    d = abs(float(np.dot(q1, q2)))
    return float(np.degrees(2.0 * np.arccos(min(1.0, d))))


@dataclass
class GuardEvent:
    kind: str        # 'bridging' | 'confirmed' | 'lost'
    t0: float        # start time [s]
    t1: float        # end time [s]
    reason: str      # 'stale' | 'frozen' | 'jump'


@dataclass
class SampleCheck:
    """Per-sample classification state (one body)."""
    good_t: float = None
    good_p: np.ndarray = None
    good_q: np.ndarray = None
    prev_p: np.ndarray = None
    same_n: int = 0
    same_since: float = None
    good_v: np.ndarray = None    # EKF2's velocity when the last good sample came (None: none)
    n_frozen: int = 0
    n_jump: int = 0
    last_bad: str = ''
    jumps: list = field(default_factory=list)   # (t, dp [m], dr [deg])

    def check(self, t, p, v, q, v_pred=None):
        """Classify one sample; returns '' (good) or the reason it is bad.

        `v` (the processor's twist) is accepted for the interface and IGNORED.
        `v_pred`: EKF2's latest velocity (ENU, fresh), or None -- the predicted
        centre (module docstring); None = the 10-21 guard exactly.
        """
        reason = ''
        if self.prev_p is not None and np.array_equal(p, self.prev_p):
            self.same_n += 1
            if self.same_since is None:
                self.same_since = t
        else:
            self.same_n = 0
            self.same_since = None
        self.prev_p = p.copy()
        if self.same_n + 1 >= FROZEN_N and self.same_since is not None and \
                t - self.same_since > FROZEN_S:
            reason = 'frozen'
        elif self.good_t is not None:
            dt = t - self.good_t
            gate_p = JUMP_P + GATE_V * dt + 0.5 * GATE_A * dt * dt
            gate_r = JUMP_R_DEG + GATE_W_DEG * dt
            dp = float(np.linalg.norm(p - self.good_p))
            if v_pred is not None and dp > gate_p:
                va = v_pred if self.good_v is None else 0.5 * (self.good_v + v_pred)
                sp = float(np.linalg.norm(va))
                if sp > PRED_V_MIN:
                    if sp > PRED_V_MAX:
                        va = va * (PRED_V_MAX / sp)
                    dp = min(dp, float(np.linalg.norm(p - (self.good_p + va * dt))))
            dr = quat_angle_deg(q, self.good_q)
            if dp > gate_p or dr > gate_r:
                reason = 'jump'
                self.jumps.append((t, dp, dr))
        if reason:
            if reason == 'frozen':
                self.n_frozen += 1
            else:
                self.n_jump += 1
            self.last_bad = reason
            return reason
        self.good_t, self.good_p, self.good_q = t, p.copy(), q.copy()
        self.good_v = None if v_pred is None else np.array(v_pred, float)
        return ''


def ekf_velocity_at(ekf, t):
    """EKF2's latest velocity at arrival time t, or None (none yet / stale / NaN).
    ekf = (t_e [M], v_e [M,3] ENU), sorted by t_e."""
    t_e, v_e = ekf
    j = int(np.searchsorted(t_e, t, side='right')) - 1
    if j < 0 or t - t_e[j] > EKF_STALE_S or not np.all(np.isfinite(v_e[j])):
        return None
    return v_e[j]


def run_ladder(t_s, p, v, q, tick=0.004, t_start=None, t_end=None, ekf=None):
    """Replay the guard over one body's mocap stream.

    t_s: arrival times [s]; p, v: ENU position / velocity [N,3]; q: [N,4] w,x,y,z.
    Evaluates the ladder on a `tick` grid (the flight law's 250 Hz) between
    t_start and t_end. ekf = (t_e, v_e): EKF2's velocity stream for the
    predicted centre (None = the 10-21 guard). Returns (events, check, max_gap).
    """
    t_start = t_s[0] if t_start is None else t_start
    t_end = t_s[-1] if t_end is None else t_end
    chk = SampleCheck()
    events = []
    state = 'mocap'
    ep_start = None
    ep_reason = ''
    ep_confirmed = False
    ep_lost = False
    recover_since = None
    k = 0
    n = len(t_s)
    latest_bad = False
    for t in np.arange(t_start, t_end, tick):
        while k < n and t_s[k] <= t:
            vp = ekf_velocity_at(ekf, t_s[k]) if ekf is not None else None
            r = chk.check(t_s[k], p[k], v[k], q[k], vp)
            latest_bad = bool(r)
            if r:
                recover_since = None
            elif ep_confirmed and recover_since is None:
                recover_since = t_s[k]
            k += 1
        if chk.good_t is None:
            continue
        age = t - chk.good_t
        bad_now = latest_bad or age > T_BRIDGE
        if state == 'mocap':
            if bad_now:
                state = 'bridging'
                ep_start = t
                ep_reason = chk.last_bad if latest_bad else 'stale'
                ep_confirmed = ep_lost = False
                recover_since = None
        if state == 'bridging':
            if age > T_CONFIRM and not ep_confirmed:
                ep_confirmed = True
                events.append(GuardEvent('confirmed', ep_start, t, ep_reason))
            if age > T_LOST and not ep_lost:
                ep_lost = True
                events.append(GuardEvent('lost', ep_start, t, ep_reason))
            done = (not bad_now) and (not ep_confirmed or
                                      (recover_since is not None and t - recover_since >= T_RECOVER))
            if done:
                events.append(GuardEvent('bridging', ep_start, t, ep_reason))
                state = 'mocap'
    if state == 'bridging':
        events.append(GuardEvent('bridging', ep_start, t_end, ep_reason))
    gaps = np.diff(t_s[(t_s >= t_start) & (t_s <= t_end)])
    return events, chk, (float(gaps.max()) if len(gaps) else 0.0)


def self_test():
    """Synthetic checks of every branch (no data needed)."""
    rate = 120.0
    t = np.arange(0.0, 20.0, 1.0 / rate)
    p = np.c_[0.2 * np.sin(0.5 * t), 0.1 * t, 1.0 + 0.0 * t]
    v = np.c_[0.1 * np.cos(0.5 * t), 0.1 + 0.0 * t, 0.0 * t]
    q = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    ev, _, _ = run_ladder(t, p, v, q)
    assert not ev, f'clean stream raised {ev}'
    # 1) a one-sample flip: bridging once, never confirmed
    p1, q1 = p.copy(), q.copy()
    i = 600
    p1[i] += [0.18, 0.0, 0.0]
    q1[i] = [np.cos(np.radians(55)), 0.0, 0.0, np.sin(np.radians(55))]
    ev, chk, _ = run_ladder(t, p1, v, q1)
    assert [e.kind for e in ev] == ['bridging'] and ev[0].reason == 'jump', ev
    # 2) a 2.84 s freeze with fresh stamps: confirmed, lost, then recovered
    p2 = p.copy()
    a, b = 1200, 1200 + int(2.84 * rate)
    p2[a:b] = p2[a - 1]
    ev, _, _ = run_ladder(t, p2, v, q)
    kinds = [e.kind for e in ev]
    assert kinds == ['confirmed', 'lost', 'bridging'] and ev[0].reason == 'frozen', ev
    # 3) a 0.10 s gap: bridging only; a 0.40 s gap: confirmed, not lost
    for gap, want in ((0.10, ['bridging']), (0.40, ['confirmed', 'bridging'])):
        keep = ~((t > 5.0) & (t < 5.0 + gap))
        ev, _, _ = run_ladder(t[keep], p[keep], v[keep], q[keep])
        assert [e.kind for e in ev] == want, (gap, ev)
    # 4) repeated frames in pairs are not a freeze
    p4 = p.copy()
    p4[1::2] = p4[0:-1:2]
    ev, _, _ = run_ladder(t, p4, v, q)
    assert not ev, ev
    # 5) the 10-29 case: a body at 3.5 m/s (29 mm per sample, the band is 28 mm).
    #    Without the prediction the first fast sample is rejected and the rest
    #    never get back in (CONFIRMED, LOST); with EKF2's velocity, nothing.
    #    And a flip in the middle of that run is still rejected.
    t5 = np.arange(0.0, 6.0, 1.0 / rate)
    vx = np.where(t5 < 2.0, 0.0, np.minimum(3.5, 3.5 * (t5 - 2.0) / 0.3))
    p5 = np.c_[np.cumsum(vx) / rate, 0.05 * t5, 1.0 + 0.0 * t5]   # never bit-identical
    v5 = np.c_[vx, 0.05 + 0.0 * t5, 0.0 * t5]
    q5 = np.tile([1.0, 0.0, 0.0, 0.0], (len(t5), 1))
    ekf = (t5 - 0.004, v5)           # EKF2 a little ahead of the mocap sample
    ev, _, _ = run_ladder(t5, p5, v5, q5)
    assert [e.kind for e in ev][:2] == ['confirmed', 'lost'], ev
    ev, _, _ = run_ladder(t5, p5, v5, q5, ekf=ekf)
    assert not ev, ev
    p6, q6 = p5.copy(), q5.copy()
    i = int(3.0 * rate)
    p6[i] += [0.0, 0.18, 0.0]                         # sideways flip, attitude kept
    ev, _, _ = run_ladder(t5, p6, v5, q6, ekf=ekf)
    assert [e.kind for e in ev] == ['bridging'] and ev[0].reason == 'jump', ev
    p7 = p5.copy()
    p7[i] += [0.12, 0.0, 0.0]                         # a flip ALONG the motion
    ev, _, _ = run_ladder(t5, p7, v5, q5, ekf=ekf)
    assert [e.kind for e in ev] == ['bridging'] and ev[0].reason == 'jump', ev
    # 6) a WRONG EKF2 velocity never rejects what the 10-21 guard accepts
    ev, _, _ = run_ladder(t, p, v, q, ekf=(t, -5.0 * np.ones_like(v)))
    assert not ev, ev
    # 7) below PRED_V_MIN the verdicts are the 10-21 guard's: a 29 mm one-sample
    #    jump at 0.5 m/s is rejected with or without EKF2's velocity
    p8 = p.copy()
    p8[900] += [0.0285, 0.0, 0.0]
    ekf_slow = (t, np.tile([0.5, 0.0, 0.0], (len(t), 1)))
    for e in (None, ekf_slow):
        ev, _, _ = run_ladder(t, p8, v, q, ekf=e)
        assert [x.kind for x in ev] == ['bridging'] and ev[0].reason == 'jump', ev
    print('guard_reference self-test: OK (clean, flip, freeze, gaps, paired frames, '
          'fast body + predicted centre, flips while fast, wrong EKF2 velocity, slow = 10-21)')


if __name__ == '__main__':
    self_test()
