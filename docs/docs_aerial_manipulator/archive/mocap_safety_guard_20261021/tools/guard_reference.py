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
    n_frozen: int = 0
    n_jump: int = 0
    last_bad: str = ''
    jumps: list = field(default_factory=list)   # (t, dp [m], dr [deg])

    def check(self, t, p, v, q):
        """Classify one sample; returns '' (good) or the reason it is bad.

        `v` (the processor's twist) is accepted for the interface and IGNORED.
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
        return ''


def run_ladder(t_s, p, v, q, tick=0.004, t_start=None, t_end=None):
    """Replay the guard over one body's mocap stream.

    t_s: arrival times [s]; p, v: ENU position / velocity [N,3]; q: [N,4] w,x,y,z.
    Evaluates the ladder on a `tick` grid (the flight law's 250 Hz) between
    t_start and t_end. Returns (events, check, max_gap).
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
            r = chk.check(t_s[k], p[k], v[k], q[k])
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
    print('guard_reference self-test: OK (clean, flip, freeze, gaps, paired frames)')


if __name__ == '__main__':
    self_test()
