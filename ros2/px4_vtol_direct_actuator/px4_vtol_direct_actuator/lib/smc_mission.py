import time, math, sys, statistics, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference, Mocap
from std_msgs.msg import String
CID = sys.argv[1] if len(sys.argv) > 1 else "geometric"
# Far target so the vehicle actually reaches wing-borne cruise speed (>~13 m/s) and the
# front rotors visibly TILT to the cruise config on the way out.
TX, TY, CRZ = 50.0, 0.0, 3.0
class M(Node):
    def __init__(s):
        super().__init__("smc_mission")
        s.pub = s.create_publisher(PositionControllerReference, "/uav_0/fsc_autopilot_ros2/position_controller/reference", 10)
        s.act = s.create_publisher(String, "/uav_0/ground_station/active_controller", 10)
        s.create_subscription(Mocap, "/uav_0/mocap", s.on_m, qos_profile_sensor_data)
        s.p = None; s.t0 = time.time(); s.last = -1; s.hov = []; s.fly = []; s.zmax = 0
        s.pp = None; s.tp = None; s.vmaxh = 0.0            # peak horizontal speed (confirms cruise)
        s.create_timer(1/20.0, s.tick)
    def on_m(s, m):
        s.p = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
        now = time.time()
        if s.pp is not None and s.tp is not None and now > s.tp:
            dt = now - s.tp
            vh = math.hypot(s.p[0]-s.pp[0], s.p[1]-s.pp[1]) / dt
            s.vmaxh = max(s.vmaxh, vh)
        s.pp = s.p; s.tp = now
    def tick(s):
        s.act.publish(String(data=CID))
        if s.p is None: return
        t = time.time() - s.t0
        if   t < 8:  tx, ty, tz, ph = 0., 0., 1.5, "HOVER0"
        elif t < 40: tx, ty, tz, ph = TX, TY, CRZ, "FLY"      # accel to cruise, TILT transition, cruise
        elif t < 52: tx, ty, tz, ph = TX, TY, CRZ, "HOVER@"   # decel + hover at the spot
        elif t < 68: tx, ty, tz, ph = TX, TY, 0.12, "LAND"    # land in hover mode
        else: raise SystemExit
        m = PositionControllerReference(); m.position.x = tx; m.position.y = ty; m.position.z = tz; m.yaw = 0.0
        s.pub.publish(m)
        s.zmax = max(s.zmax, s.p[2])
        if ph == "HOVER@": s.hov.append(math.hypot(s.p[0]-TX, s.p[1]-TY) + abs(s.p[2]-CRZ))
        if ph == "FLY":    s.fly.append(math.hypot(s.p[0]-tx, s.p[1]-ty))
        if int(t) != s.last:
            s.last = int(t)
            print(f"[{CID}] t={t:4.0f} {ph:7s} tgt=({tx:.0f},{ty:.0f},{tz:.1f}) veh=({s.p[0]:6.2f},{s.p[1]:5.2f},{s.p[2]:5.2f})", flush=True)
rclpy.init(); n = M()
try: rclpy.spin(n)
except (KeyboardInterrupt, SystemExit): pass
if n.p:
    eh = math.hypot(n.p[0]-TX, n.p[1]-TY)
    hov = statistics.mean(n.hov) if n.hov else float('nan')
    fly = statistics.mean(n.fly) if n.fly else float('nan')
    tumbled = "YES" if n.zmax > 8.0 else "no"   # climbed away = unstable
    print(f"[{CID}] RESULT land=({n.p[0]:.2f},{n.p[1]:.2f},{n.p[2]:.2f}) land_err={eh:.2f}m hover@spot={hov:.2f}m fly_track={fly:.2f}m peak_speed={n.vmaxh:.1f}m/s tumbled={tumbled}", flush=True)
rclpy.shutdown()
