#!/usr/bin/env python3
"""Check the valve's mocap body (obj_0) before a valve-turning flight (2026-10-27; 2026-10-28:
the markers' centroid as the pivot).

The planner (push_pull_valve_axis_offset, 2026-10-28) takes obj_0 = the rigid body Motive makes
from the rotor's markers, with its DEFAULT pivot (the markers' centroid) and its axes the rotor's
(+x along the beam toward the fin, z up -- create it with the fin pointing along the room's +x on
a level table). The bearing axis at the plate-bottom plane is then obj_0 + Rz(yaw) * the axis
offset: on Valve_Assembly_v2.stl's five markers [-0.0720, 0.0093, -0.0851] m. Get places the turn,
the table and the valve's keep-out from that point, and the grasp point is it + push_pull_ee_offset.

Turn the beam slowly by hand through 60 deg or more while this runs. It MEASURES the axis offset
(in obj_0's frame: the horizontal part; the vertical part is the CAD's, a turn cannot see it) and
reports:
  - the measured offset against the expected one (--axis-offset): within 3 mm = OK; else type the
    measured x, y into the arm GS's Axis Offset row (the markers sit off their CAD places)
  - the frame's TURN about z against the beam: the measured offset's direction against the
    expected one. Within 3 deg = OK; else re-create the rigid body with the fin along the room's +x
    (obj_0 +x must run along the beam -- the drone's heading at the fin is obj_0's yaw + 90)
  - the axis's tilt from vertical (level the plate if > 1.5 deg) and obj_0's z against the shaft:
    the planner's rotor-tilt guard reads a z off the shaft as FALSE tilt during the turn, so the
    body must be created on a LEVEL plate (within ~1 deg of the shaft for a 90 deg turn, 2 deg guard)
  - the axis point's height (= the table top the planner will use) and, with --table-top, the error
  - obj_0's yaw now: point the fin at the vehicle's start; the yaw must then read the bearing from
    the axis to the start (the planner refuses a beam > 45 deg off it)
--axis-offset 0,0,0 checks a body defined with its pivot ON the axis (the pre-2026-10-28 rule).

Live (ROS sourced):     /usr/bin/python3 valve_body_check.py [--topic /obj_0/mocap] [--seconds 20] [--table-top 0.80]
Offline (an extract):   /usr/bin/python3 valve_body_check.py --npz spin.npz     (archive/valve_damper_spin_20261027)
"""
import argparse
import sys
import numpy as np


def quat_to_R(q):                                   # q: (n, 4) x, y, z, w
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


AXIS_OFFSET_CAD = (-0.0720, 0.0093, -0.0851)   # minus the v2 valve's five-marker centroid (stl_to_usda_valve.py)


def analyse(p, q, table_top=None, axis_offset=AXIS_OFFSET_CAD, turn_deg=90.0, tilt_limit=2.0):
    """p: (n, 3) pivot positions [m], q: (n, 4) quaternions x, y, z, w; axis_offset: the planner's
    push_pull_valve_axis_offset [m, obj_0 frame]. Returns (ok, lines, measured offset)."""
    R = quat_to_R(q / np.linalg.norm(q, axis=1, keepdims=True))
    yaw = np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    span = np.degrees(yaw.max() - yaw.min())
    out = [f"samples {len(p)}, the beam turned through {span:.0f} deg"]
    if span < 45.0:
        return False, out + ["NOT ENOUGH ROTATION: turn the beam by hand through 60 deg or more while this runs"], None
    # the rotation axis: the mean direction of the rotation vectors of large relative rotations
    idx = np.linspace(0, len(R) - 1, min(len(R), 200)).astype(int)
    axes = []
    for i in idx[::4]:
        for j in idx:
            M = R[j] @ R[i].T
            ang = np.arccos(np.clip((np.trace(M) - 1) / 2, -1, 1))
            if ang > np.radians(30):
                v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])
                v /= np.linalg.norm(v)
                axes.append(v * np.sign(v[2]))
    n = np.mean(axes, 0); n /= np.linalg.norm(n)
    tilt = np.degrees(np.arccos(n[2]))
    ok = True
    out.append(f"rotation axis tilt {tilt:.2f} deg from vertical (toward azimuth "
               f"{np.degrees(np.arctan2(n[1], n[0])):.0f} deg)" + ("  -- LEVEL THE PLATE (the yaw-only axis offset "
               f"is then off by ~{85.1 * np.tan(np.radians(tilt)):.0f} mm)" if tilt > 1.5 else "  OK"))
    # THE AXIS LINE in the room: the point of the body that does not move, p_i + R_i d = c for
    # every sample (least squares; along the axis it is unobservable), and the direction n
    k = np.arange(0, len(p), max(1, len(p) // 3000))
    A = np.zeros((3 * len(k), 6)); b = np.zeros(3 * len(k))
    for r, i in enumerate(k):
        A[3 * r:3 * r + 3, :3] = R[i]
        A[3 * r:3 * r + 3, 3:] = -np.eye(3)
        b[3 * r:3 * r + 3] = -p[i]
    c = np.linalg.lstsq(A, b, rcond=None)[0][3:]
    # WHAT THE PLANNER COMPUTES: axis point = obj_0 + Rz(yaw) * offset, z from the offset's z.
    # So per sample: the axis line's point at obj_0's height + e_z, back in obj_0's YAW frame
    e = np.asarray(axis_offset, float)
    zt = p[:, 2] + e[2]
    on_axis = c + np.outer((zt - c[2]) / n[2], n)
    w = on_axis - p
    cy, sy = np.cos(yaw), np.sin(yaw)
    v = np.c_[cy * w[:, 0] + sy * w[:, 1], -sy * w[:, 0] + cy * w[:, 1]]      # Rz(-yaw) (axis - obj_0), xy
    d = np.r_[np.median(v, 0), e[2]]
    scatter = np.percentile(np.linalg.norm(v - d[:2], axis=1), 95)
    out.append(f"obj_0 -> the axis point, measured as the planner reads it [{d[0]:+.4f}, {d[1]:+.4f}, {d[2]:+.4f}] m "
               f"({np.hypot(d[0], d[1]) * 1e3:.1f} mm off the shaft, 95 % within {scatter * 1e3:.1f} mm); expected "
               f"(--axis-offset) [{e[0]:+.4f}, {e[1]:+.4f}, {e[2]:+.4f}] ({np.hypot(e[0], e[1]) * 1e3:.1f} mm)")
    diff = np.hypot(*(d[:2] - e[:2]))
    turned = False
    if np.hypot(e[0], e[1]) > 0.02 and np.hypot(d[0], d[1]) > 0.02:
        # an obj_0 frame turned by +theta about z sees the rotor-fixed offset turned by -theta
        turn = -np.degrees(np.arctan2(e[0] * d[1] - e[1] * d[0], e[0] * d[0] + e[1] * d[1]))
        if abs(turn) > 3.0:
            ok = False
            turned = True
            cr, sr = np.cos(np.radians(turn)), np.sin(np.radians(turn))
            rest = np.hypot(cr * d[0] - sr * d[1] - e[0], sr * d[0] + cr * d[1] - e[1])
            out.append(f"obj_0's FRAME IS TURNED: its +x points {turn:+.1f} deg from the beam (+ = counter-clockwise "
                       f"seen from above) -- re-create the rigid body "
                       f"with the FIN along the room's +x (obj_0 +x must run along the beam to the fin). Turned back, "
                       f"the markers' centroid sits {rest * 1e3:.1f} mm from the expected place")
        else:
            out.append(f"obj_0's +x points {turn:+.1f} deg from the beam  OK")
    if turned:
        pass
    elif diff > 0.003:
        ok = False
        out.append(f"THE OFFSET DIFFERS by {diff * 1e3:.1f} mm from the expected one: the markers or the pivot are not "
                   f"where the planner assumes -- type [{d[0]:.4f}, {d[1]:.4f}, {e[2]:.4f}] into the arm GS's Axis "
                   f"Offset row (push_pull_valve_axis_offset), or redefine the body" +
                   ("" if np.hypot(e[0], e[1]) > 1e-9 else " with its pivot ON the axis at the plate-bottom plane"))
    else:
        out.append(f"the offset matches the expected one to {diff * 1e3:.1f} mm  OK")
    # the axis point's height: obj_0 z + the offset's z (the plate bottom = the table top)
    z = float(np.median(zt))
    out.append(f"the axis point's height {z:.3f} m = the table top the planner will use")
    if table_top is not None:
        ez = z - table_top
        out.append(f"  vs the measured table top {table_top:.3f} m: {ez * 1e3:+.0f} mm"
                   + ("  OK" if abs(ez) <= 0.005 else "  -- the offset's z (the markers' height) is off: check the CAD"))
        ok = ok and abs(ez) <= 0.005
    # obj_0's z against the SHAFT (2026-10-28, the hardware review): the planner's rotor-tilt guard
    # (push_pull_guard_box_tilt_deg 2.0) compares obj_0's z with its z at the grasp, so a body whose
    # z is gamma off the shaft reads 2 asin(sin gamma sin(theta/2)) of FALSE tilt after a turn of
    # theta -- at the 10-27 bench's 2.2 deg it would abort ~54 deg into a 90 deg turn, clamped.
    # Motive gives a new rigid body the room's vertical: LEVEL THE PLATE WHEN CREATING THE BODY
    # (afterwards a tilted table does not matter -- body and shaft tilt together).
    zb = R[:, :, 2] @ n
    zoff = np.degrees(np.arccos(np.clip(np.median(zb), -1, 1)))
    th = np.radians(turn_deg + 15.0)                       # the turn + a 15 deg overshoot
    false_tilt = np.degrees(2.0 * np.arcsin(np.sin(np.radians(zoff)) * np.sin(0.5 * th)))
    zok = false_tilt <= 0.75 * tilt_limit
    ok = ok and zok
    out.append(f"obj_0's z axis {zoff:.2f} deg from the rotation axis: the rotor-tilt guard would read {false_tilt:.2f} deg "
               f"of false tilt after a {turn_deg:.0f}+15 deg turn (limit {tilt_limit:.1f})" +
               ("  OK" if zok else f"  -- LEVEL THE PLATE (shaft vertical) and RE-CREATE the rigid body: obj_0's z must "
                                   f"lie within {np.degrees(np.arcsin(np.sin(np.radians(0.75 * tilt_limit) / 2.0) / np.sin(0.5 * th))):.2f} deg of the shaft"))
    out.append(f"obj_0's yaw NOW {np.degrees(np.arctan2(R[-1, 1, 0], R[-1, 0, 0])):.1f} deg: with the fin pointed at the "
               f"vehicle's start it must read the bearing from the axis to the start (+x = along the beam to the fin)")
    return ok, out, d


def from_npz(path):
    d = np.load(path)
    p = np.c_[[d["vrpn__pose.position." + c] for c in "xyz"]].T
    q = np.c_[[d["vrpn__pose.orientation." + c] for c in "xyzw"]].T
    return p, q


def from_topic(topic, seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = Node("valve_body_check")
    P, Q = [], []
    if "vrpn" in topic:
        from geometry_msgs.msg import PoseStamped as T
    else:
        from fsc_autopilot_ros2_msgs.msg import Mocap as T

    def cb(m):
        P.append([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        o = m.pose.orientation
        Q.append([o.x, o.y, o.z, o.w])
    node.create_subscription(T, topic, cb, qos_profile_sensor_data)
    print(f"recording {topic} for {seconds:.0f} s -- turn the beam slowly by hand through 60 deg or more ...")
    import time
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if len(P) < 50:
        sys.exit(f"only {len(P)} samples on {topic} -- is the mocap streaming obj_0?")
    return np.array(P), np.array(Q)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/obj_0/mocap")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--table-top", type=float, default=None, help="the measured table-top height [m], optional")
    ap.add_argument("--npz", default=None, help="analyse an extracted bag instead (extract.py output)")
    ap.add_argument("--axis-offset", default=",".join(f"{v:g}" for v in AXIS_OFFSET_CAD),
                    help="the planner's push_pull_valve_axis_offset 'x,y,z' [m] (default: the v2 valve's "
                         "five-marker centroid); 0,0,0 = a body defined with its pivot ON the axis")
    ap.add_argument("--turn-deg", type=float, default=90.0, help="the planned turn (push_pull_turn_deg)")
    ap.add_argument("--tilt-limit", type=float, default=2.0, help="the rotor-tilt guard (push_pull_guard_box_tilt_deg)")
    a = ap.parse_args()
    try:
        off = [float(v) for v in a.axis_offset.split(",")]
        assert len(off) == 3
    except (ValueError, AssertionError):
        sys.exit(f"--axis-offset must be 'x,y,z' in metres, got {a.axis_offset!r}")
    p, q = from_npz(a.npz) if a.npz else from_topic(a.topic, a.seconds)
    ok, lines, _ = analyse(p, q, a.table_top, off, abs(a.turn_deg), a.tilt_limit)
    for line in lines:
        print(line)
    print("RESULT: " + ("OK -- obj_0 is defined as the planner assumes" if ok else "FIX obj_0 BEFORE FLYING"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
