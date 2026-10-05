// fig8_probe.cpp -- the planner's OWN numbers for candidate figure-8 shapes
// (2026-10-05, figure-8 comparison design). Links fsc_trajectory_planner's
// planner_lib (EeTrajectoryPlanner) and fsc_autopilot_ros2's wb_law, builds the
// T650 aerial-manipulator model exactly as the mirror yaml's planner section does
// (base_com, joint-diagonal armature), and for every candidate prints the peaks
// at time scale 1 with the bounds OPENED (what the shape demands) plus s_max at
// the given bounds.
//
//   fig8_probe A B lap_time q2_period fold q2_center q2_amp [v_max a_max w_max qdot_max] [circle]
//
// Build (after building fsc_trajectory_planner):
//   g++ -O2 -std=c++17 fig8_probe.cpp -I$HOME/ros2_ws/src/fsc_trajectory_planner/include \
//     -I$HOME/ros2_ws/install/fsc_autopilot_ros2/include -I/usr/include/eigen3 \
//     $HOME/ros2_ws/install/fsc_trajectory_planner/lib/libfsc_trajectory_planner.a \
//     $HOME/ros2_ws/install/fsc_autopilot_ros2/lib/libwb_law.a -o fig8_probe
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "fsc_trajectory_planner/ee_trajectory_planner.hpp"
#include "fsc_trajectory_planner/vehicle_model.hpp"

using namespace fsc_trajectory_planner;

int main(int argc, char ** argv)
{
  if (argc < 8) {
    std::fprintf(stderr, "usage: fig8_probe A B lap q2_period fold q2_center q2_amp [v a w qdot] [circle]\n");
    return 2;
  }
  VehicleOptions vo;
  vo.base_com = Vec3{0.0, -0.017854, 0.0};
  vo.armature_joint_diag = true;
  vo.armature << 0.010, 0.0194, 0.0097, 0.0097;
  const auto v = makeVehicleModel("t650_aerial_manipulator", vo);
  RestSpec hold;
  hold.x_b << 0.0, 0.0, 1.2;
  hold.phi = -0.5 * M_PI;
  hold.q = v->home;

  EeShape sh;
  const bool circle = argc > 12 && std::string(argv[12]) == "circle";
  sh.type = circle ? "circle" : "figure8";
  sh.fig8_a = std::atof(argv[1]);
  sh.fig8_b = std::atof(argv[2]);
  sh.radius = std::atof(argv[1]);
  sh.lap_time = std::atof(argv[3]);
  sh.laps = 1;
  EeTrajectoryOptions o;
  o.q2_period_s = std::atof(argv[4]);
  o.ee_fold_deg = std::atof(argv[5]);
  o.q2_center_deg = std::atof(argv[6]);
  o.q2_amp_deg = std::atof(argv[7]);
  o.qdot_max = 0.5;
  o.v_max = 0.30; o.a_max = 0.15; o.w_max = 0.30;
  if (argc > 11) {
    o.v_max = std::atof(argv[8]); o.a_max = std::atof(argv[9]);
    o.w_max = std::atof(argv[10]); o.qdot_max = std::atof(argv[11]);
  }
  o.tau_joint_max = 3.0;
  o.rotor_bounds = true;
  o.sigma_nd_min = v->sigma_nd_margin;
  o.beta_min_deg = v->beta_min_deg;

  // 1. what the shape demands at s = 1: every rate bound opened
  EeTrajectoryOptions open = o;
  open.time_scale = 1.0;
  open.v_max = open.a_max = open.w_max = open.qdot_max = 1e9;
  EeTrajectoryDiag d;
  try {
    EeTrajectoryPlanner::plan(*v, hold, sh, open, &d);
  } catch (const std::exception & e) {
    std::printf("PLAN(s=1, open) FAILED: %s\n", e.what());
  }
  std::printf("%s\n", d.summary.c_str());
  std::printf("s=1 peaks: |v_c| %.3f m/s  |a_c| %.3f m/s^2  |psi_dot| %.3f rad/s (%.1f deg/s)  |q_dot| %.3f rad/s (%.1f deg/s)"
              "  tau_j %.3f N.m  rotor %.2f..%.2f N  sigma_nd %.3f  T %.2f s (lap %.2f)  q_min [%.1f %.1f %.1f %.1f] q_max [%.1f %.1f %.1f %.1f]\n",
              d.peak_v, d.peak_a, d.peak_w, d.peak_w * 180 / M_PI, d.peak_qdot, d.peak_qdot * 180 / M_PI,
              d.peak_tau_joint, d.rotor_lo, d.rotor_hi, d.min_sigma_nd, d.T_total, d.T_lap,
              d.q_min_deg(0), d.q_min_deg(1), d.q_min_deg(2), d.q_min_deg(3),
              d.q_max_deg(0), d.q_max_deg(1), d.q_max_deg(2), d.q_max_deg(3));
  // 2. s_max at the given bounds
  std::string why;
  const double smax = EeTrajectoryPlanner::maxTimeScale(*v, hold, sh, o, 6.0, 0.01, &why);
  std::printf("s_max at v %.2f a %.2f w %.2f qdot %.2f: %.3f %s\n", o.v_max, o.a_max, o.w_max, o.qdot_max,
              smax, why.c_str());
  if (smax > 0) {
    EeTrajectoryOptions oo = o; oo.time_scale = std::min(smax, 6.0);
    EeTrajectoryDiag dd;
    try { EeTrajectoryPlanner::plan(*v, hold, sh, oo, &dd); } catch (...) {}
    std::printf("  at s_max: %s\n", dd.summary.c_str());
    // which bound binds just above s_max
    oo.time_scale = smax * 1.03;
    try { EeTrajectoryPlanner::plan(*v, hold, sh, oo, &dd); } catch (const std::exception & e) {
      std::printf("  just above: %s\n", e.what());
    }
  }
  // optional dump of the plan at s = 1 (bounds opened): DUMP=path.csv
  if (const char * dump = std::getenv("DUMP")) {
    double yaw0 = 0.0;
    if (const char * y = std::getenv("YAW_DEG")) {yaw0 = std::atof(y) * M_PI / 180.0;}
    RestSpec h2 = hold;
    h2.phi = yaw0 - 0.5 * M_PI;
    EeTrajectoryDiag dd;
    const auto tr = EeTrajectoryPlanner::plan(*v, h2, sh, open, &dd);
    FILE * f = std::fopen(dump, "w");
    std::fprintf(f, "t,ex,ey,ez,cx,cy,cz,vcx,vcy,vcz,acx,acy,acz,b1x,b1y,bex,bey,q1,q2,q3,q4,qd1,qd2,qd3,qd4\n");
    for (double t = 0.0; t <= tr->duration() + 1e-9; t += 0.02) {
      const WbReference r = tr->eval(t);
      std::fprintf(f, "%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        t, r.r_ed(0), r.r_ed(1), r.r_ed(2), r.x_cd(0), r.x_cd(1), r.x_cd(2),
        r.x_cd_dot(0), r.x_cd_dot(1), r.x_cd_dot(2), r.x_cd_ddot(0), r.x_cd_ddot(1), r.x_cd_ddot(2),
        r.b1_d(0), r.b1_d(1), r.b1_de(0), r.b1_de(1),
        r.q_d(0), r.q_d(1), r.q_d(2), r.q_d(3), r.qdot_d(0), r.qdot_d(1), r.qdot_d(2), r.qdot_d(3));
    }
    std::fclose(f);
    std::printf("dumped %s (T %.2f s)\n", dump, tr->duration());
  }
  return 0;
}
