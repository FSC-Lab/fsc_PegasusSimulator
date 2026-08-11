function ctrl = controller(t, X, dyn, ref, params, acc, p_hat) %#ok<INUSL>
% acc (optional): struct with fields .omega_0_dot (3x1) and .qddot (nx1), the
%   closed-loop accelerations supplied by closed_loop_dynamics (predictor-
%   corrector) for the exact omega_e_dot feedforward.  If omitted, a causal
%   finite difference is used as a fallback.
% p_hat (optional): the GENERALIZED-MOMENTUM OBSERVER (GMO) state
%   p_hat in R^{6+n}, an *estimate* of the transformed momentum p = M_tilde*xi
%   that is carried by the augmented ODE state and integrated over time (see
%   closed_loop_dynamics / main_sim).  The GMO turns the measured momentum
%   mismatch (p - p_hat) into an estimate of the TRANSFORMED end-effector
%   disturbance
%          d_e_hat = K_o (p - p_hat) = [d_t_hat; d_r_hat; d_rho_hat]
%   (PDF "Control -- Disturbance Observer (GMO)").  The controller
%     * cancels  d_t_hat  in the force law f_d / thrust u1   (translation),
%     * cancels  d_r_hat  in the base-moment law u2          (rotation),
%     * injects  F_hat_y = (J_y^#)^T d_e_hat  into the arm law u3 through the
%       inertia-shaping term -(Lambda_y M_y^{-1} - I) F_hat_y (arm), which is
%       ACTIVE only when a desired task inertia M_y ~= Lambda_y is requested.
%   and returns ctrl.p_hat_dot = C_tilde^T xi - g_tilde + u + d_e_hat, the GMO
%   state derivative, which the caller appends to Xdot.  Per the flowchart,
%   p_hat_dot is formed AFTER u is computed (it depends on u).  If p_hat is
%   omitted/empty the observer is OFF: d_e_hat = 0 (nominal, no disturbance
%   compensation) and ctrl.p_hat_dot = 0.
%CONTROLLER  Cartesian impedance controller (+GMO) for the aerial manipulator.
%
%   ctrl = controller(t, X, dyn, ref, params, acc, p_hat) implements the
%   controller flowchart (Control flowchart.pdf):
%       Control -- Transformed State
%       Control -- Disturbance Observer (GMO)
%       Control -- Translation
%       Control -- Rotation
%       Control -- Arm
%
%   INPUTS
%       t      : time (used only by the finite-difference feedforward fallback)
%       X      : full state  [r_0; vec(R_0); q; v_0; omega_0; qdot]
%       dyn    : struct from dynamics() (M_tilde, C_tilde, g_tilde, M_r, C_r,
%                C_rp, C_p, A, N1, J_y, J_y_dot, Lambda_y, J_1y, J_2y, J_3y,
%                r_0c_0, r_0e_0, R_e_0, omega_0e_0, ...)
%       ref    : reference struct (plan_locked_trajectory / plan_compatible_trajectory)
%       params : model parameters (n, m_i, g, ...) AND all controller gains,
%                which are defined/tuned in main_sim (Section 1c) and only READ
%                here (this file keeps no gain defaults of its own):
%                  params.k_x, k_v, k_R, k_w   scalar tracking gains
%                  params.M_r_d (3x3)          desired rotational inertia
%                  params.D_y, params.K_y (4x4) task-space damping / stiffness
%                  params.M_y (4x4 or [])      desired task inertia ([] -> natural)
%                  params.K_o (6+n)            GMO observer gain
%       acc    : (optional) closed-loop accelerations for omega_e_dot feedforward
%       p_hat  : (optional) GMO momentum-estimate state (6+n x 1)
%
%   OUTPUT
%       ctrl   : struct with u1 (scalar thrust), u2 (3x1 base torque),
%                u3 (n x1 joint torques), stacked u, the GMO derivative
%                p_hat_dot and disturbance estimate d_e_hat, plus debug fields.
%
%   Requires helpers on the path:  hat.  (vee is provided locally.)

% ========================================================================
% Constants + gains  (ALL gains are set/tuned in main_sim, Section 1c)
% ========================================================================
% ---------------- Model constants ----------------------------------------
e1 = [1;0;0];  e2 = [0;1;0];  e3 = [0;0;1]; %#ok<NASGU>
n  = params.n;
g  = params.g;
m  = sum(params.m_i);          % total system mass

% ---------------- Controller gains (read from params) --------------------
% Rigid-body tracking:
k_x   = params.k_x;            % CoM position gain      (scalar, acts on 3-vectors)
k_v   = params.k_v;            % CoM velocity gain
k_R   = params.k_R;            % attitude gain
k_w   = params.k_w;            % angular-rate gain
M_r_d = params.M_r_d;          % desired rotational inertia (3x3)
% Task-space impedance behaviour (damping, stiffness, desired inertia):
D_y   = params.D_y;            % task-space damping    (4x4: 3 position + 1 heading)
K_y   = params.K_y;            % task-space stiffness  (4x4)
M_y   = params.M_y;            % desired task inertia  (4x4 SPD, or [] -> natural
                               %   inertia Lambda_y; resolved in the Arm section)
% GMO disturbance observer:
K_o   = params.K_o;            % observer gain (6+n): d_e_hat_dot = K_o(d_e-d_e_hat)

% ---------------- Unpack state -------------------------------------------
r_0     = X(1:3);
R_0     = reshape(X(4:12), 3, 3);
v_0     = X(13+n:15+n);
omega_0 = X(16+n:18+n);
qdot    = X(19+n:18+2*n);
V       = [v_0; omega_0; qdot];
om_hat  = hat(omega_0);

% ---------------- Unpack dyn ---------------------------------------------
A          = dyn.A;
N1         = dyn.N1;
r_0c_0     = dyn.r_0c_0;
r_0e_0     = dyn.r_0e_0;
R_e_0      = dyn.R_e_0;
M_r        = dyn.M_r;
C_r        = dyn.C_r;
C_rp       = dyn.C_rp;
C_p        = dyn.C_p;
J_y        = dyn.J_y;
J_y_dot    = dyn.J_y_dot;
Lambda_y   = dyn.Lambda_y;
J_1y       = dyn.J_1y;     % (J_y^#)^T column block 1:3   (4 x 3)
J_2y       = dyn.J_2y;     % (J_y^#)^T column block 4:6   (4 x 3)
J_3y       = dyn.J_3y;     % (J_y^#)^T column block 7:end (4 x n)
omega_0e_0 = dyn.omega_0e_0;
J_we       = dyn.J_q_omega_e;       % J_{omega_e}^q          (3 x n)
J_we_dot   = dyn.J_q_dot_omega_e;   % d/dt J_{omega_e}^q     (3 x n)  [must be exported]
% Whole-body transformed matrices (needed by the GMO momentum observer)
M_tilde    = dyn.M_tilde;           % (6+n)x(6+n)  blkdiag(mI3, M_r, M_rho)
C_tilde    = dyn.C_tilde;           % (6+n)x(6+n)  transformed Coriolis
g_tilde    = dyn.g_tilde;           % (6+n)x1      [m g e3; 0; 0]

% ========================================================================
% Control -- Transformed State
% ========================================================================
% T = [ R0, -R0*hat(r_0c^0), R0*A ; 0, I3, 0 ; 0, N1, In ],  xi = T*V
T = [ R_0,         -R_0*hat(r_0c_0),  R_0*A          ;
      zeros(3,3),   eye(3),           zeros(3,n)     ;
      zeros(n,3),   N1,               eye(n)        ];
xi      = T * V;            % xi = [xc_dot; omega_0; rho]
xc_dot  = xi(1:3);
rho     = xi(7:6+n);

% ========================================================================
% Control -- Disturbance Observer (GMO)
% ========================================================================
% Generalized-momentum observer on the TRANSFORMED dynamics
%     M_tilde xi_dot + C_tilde xi + g_tilde = u + d_e ,   p = M_tilde xi .
% Using the passivity identity M_tilde_dot = C_tilde + C_tilde^T, the true
% momentum obeys  p_dot = C_tilde^T xi - g_tilde + u + d_e , so with the
% observer copy  p_hat_dot = C_tilde^T xi - g_tilde + u + d_e_hat  and the
% innovation  d_e_hat = K_o (p - p_hat), the estimate is a first-order low-pass
% of the true disturbance:  d_e_hat_dot = K_o (d_e - d_e_hat).  The gain K_o is
% set in the TUNABLE GAINS block at the top of this file.
p = M_tilde * xi;                            % transformed momentum (6+n x1)
gmo_on = (nargin >= 7) && ~isempty(p_hat);
if gmo_on
    d_e_hat = K_o * (p - p_hat);             % disturbance estimate (6+n x1)
else
    d_e_hat = zeros(6+n, 1);                 % observer OFF -> nominal behaviour
end
d_t_hat   = d_e_hat(1:3);                    % translational -> cancelled in u1
d_r_hat   = d_e_hat(4:6);                    % rotational    -> cancelled in u2
d_rho_hat = d_e_hat(7:6+n);                  % arm           -> used in u3 via F_hat_y

% ========================================================================
% Control -- Translation
% ========================================================================
x_c  = r_0 + R_0 * r_0c_0;                 % system CoM in inertial frame
e_x  = x_c    - ref.x_cd;
e_vx = xc_dot - ref.x_cd_dot;

% Desired CoM force (Image 5):  f_d = -k_x e_x - k_v e_vx + m xc_dd_d + m g e3 - d_t_hat
% The estimated translational disturbance d_t_hat is cancelled here, so the
% realized thrust automatically carries the payload weight once the GMO has
% converged.  NOTE: only the zeroth-order force is compensated; f_d_dot /
% f_d_ddot below (used to plan the attitude reference) keep their reference-only
% form, since for a (near-)constant inertial payload weight d_t_hat_dot ~ 0.
f_d = -k_x*e_x - k_v*e_vx + m*ref.x_cd_ddot + m*g*e3 - d_t_hat;   % desired force (3x1)
u1  = f_d' * (R_0*e3);                                   % thrust (scalar)

% higher CoM derivatives.  The exact plant acceleration carries the disturbance:
%   m xc_ddot = u1 R0 e3 - m g e3 + d_t   ->   use d_t_hat as its best estimate.
xc_ddot = (u1/m)*(R_0*e3) - g*e3 + (1/m)*d_t_hat;
e_ax    = xc_ddot - ref.x_cd_ddot;

f_d_dot = -k_x*e_vx - k_v*e_ax + m*ref.x_cd_d3;          % (3x1)
u1_dot  = f_d_dot'*(R_0*e3) + f_d'*(R_0*om_hat*e3);      % scalar thrust rate

xc_d3 = (u1_dot/m)*(R_0*e3) + (u1/m)*(R_0*om_hat*e3);
e_jx  = xc_d3 - ref.x_cd_d3;

f_d_ddot = -k_x*e_ax - k_v*e_jx + m*ref.x_cd_d4;         % (3x1)

% ========================================================================
% Control -- Rotation
% ========================================================================
nf = norm(f_d);
% ---- first level ----
b3c = f_d / nf;
P1  = eye(3) - b3c*b3c';
s   = P1 * ref.b1_d;
b3c_dot = (1/nf) * P1 * f_d_dot;

ns  = norm(s);
b1c = s / ns;
P1_dot = -(b3c_dot*b3c' + b3c*b3c_dot');
P2     = eye(3) - b1c*b1c';
s_dot  = P1_dot*ref.b1_d + P1*ref.b1_d_dot;
b1c_dot = (1/ns) * P2 * s_dot;
P2_dot  = -(b1c_dot*b1c' + b1c*b1c_dot');

b2c = cross(b3c, b1c);
R0c = [b1c, b2c, b3c];
b2c_dot = cross(b3c_dot, b1c) + cross(b3c, b1c_dot);
R0c_dot = [b1c_dot, b2c_dot, b3c_dot];
omega_0c = vee(R0c' * R0c_dot);

% ---- second level ----
b3c_ddot = (1/nf)*(P1_dot*f_d_dot + P1*f_d_ddot) ...
           - (b3c'*f_d_dot)/(nf^2) * P1*f_d_dot;
P1_ddot  = -(b3c_ddot*b3c' + 2*b3c_dot*b3c_dot' + b3c*b3c_ddot');
s_ddot   = P1_ddot*ref.b1_d + 2*P1_dot*ref.b1_d_dot + P1*ref.b1_d_ddot;
b1c_ddot = (1/ns)*(P2_dot*s_dot + P2*s_ddot) ...
           - (b1c'*s_dot)/ns * b1c_dot;
b2c_ddot = cross(b3c_ddot, b1c) + 2*cross(b3c_dot, b1c_dot) + cross(b3c, b1c_ddot);
R0c_ddot = [b1c_ddot, b2c_ddot, b3c_ddot];
omega_0c_dot = vee(R0c_dot'*R0c_dot + R0c'*R0c_ddot);

% ---- errors and torque ----
e_R = 0.5 * vee(R0c'*R_0 - R_0'*R0c);
e_w = omega_0 - R_0'*R0c*omega_0c;

% Base-moment law (Image 7), with the estimated rotational disturbance d_r_hat
% cancelled:
u2 = M_r * ( R_0'*R0c*omega_0c_dot ...
            - om_hat*R_0'*R0c*omega_0c ...
            - M_r_d \ (k_R*e_R + k_w*e_w) ) ...
     + C_r*omega_0 + C_rp*rho - d_r_hat;

% ========================================================================
% Control -- Arm
% ========================================================================
% End-effector pose
R_e   = R_0 * R_e_0;                         % end-effector orientation
R_0e  = R_e_0';                              % R_0^e = (R_e^0)^T
r_e   = r_0 + R_0 * r_0e_0;                  % end-effector position

% End-effector BODY twist (in {e}):  omega_e = R_0^e (omega_0 + J_we*qdot)
omega_e     = R_0e * (omega_0 + omega_0e_0);
omega_e_hat = hat(omega_e);

% ---- joint & base angular accelerations (feedforward for omega_e_dot) ----
% qddot and omega_0_dot are closed-loop accelerations: NOT in the state, and
% they appear only as feedforward in omega_e_dot (Image 2).  Preferred path:
% closed_loop_dynamics supplies them via `acc` (predictor-corrector), giving
% the exact value.  Fallback (standalone calls): causal finite difference,
% zero on the first call / solver reset.
persistent t_prev qdot_prev omega0_prev
if nargin >= 6 && ~isempty(acc)
    qddot       = acc.qddot;
    omega_0_dot = acc.omega_0_dot;
else
    if isempty(t_prev) || t <= t_prev
        qddot       = zeros(n,1);            % first call / solver reset
        omega_0_dot = zeros(3,1);
    else
        dt          = t - t_prev;
        qddot       = (qdot    - qdot_prev )/dt;
        omega_0_dot = (omega_0 - omega0_prev)/dt;
    end
    t_prev = t;  qdot_prev = qdot;  omega0_prev = omega_0;
end

% omega_e_dot  (Image 2):  R_0^e( wdot_0 + J_we*qddot + Jdot_we*qdot + w0_hat*J_we*qdot )
% note J_we*qdot == omega_0e_0, so the transport term is om_hat*omega_0e_0
omega_e_dot     = R_0e * ( omega_0_dot + J_we*qddot ...
                           + J_we_dot*qdot + om_hat*omega_0e_0 );
omega_e_dot_hat = hat(omega_e_dot);

% End-effector body axes
b1e = R_e*e1;   b2e = R_e*e2;   b3e = R_e*e3; %#ok<NASGU>

% Task velocity ydot = J_y*xi = [r_e_dot; omega_3e]
ydot     = J_y * xi;
r_e_dot  = ydot(1:3);
omega_3e = omega_e' * e3;                     % = e3^T omega_e  (== ydot(4))

% ---- commanded EE heading frame  (first level) ----
P1e  = eye(3) - b3e*b3e';
s_e  = P1e * ref.b1_de;
b3e_dot = R_e * omega_e_hat * e3;            % = R_e*hat(omega_e)*e3   (Image 1)
nse  = norm(s_e);
b1ec = s_e / nse;
P1e_dot = -(b3e_dot*b3e' + b3e*b3e_dot');
P2e     = eye(3) - b1ec*b1ec';
s_e_dot = P1e_dot*ref.b1_de + P1e*ref.b1_de_dot;
b1ec_dot = (1/nse) * P2e * s_e_dot;
P2e_dot  = -(b1ec_dot*b1ec' + b1ec*b1ec_dot');

% ---- commanded EE heading frame  (second level) ----
b3e_ddot = R_e * (omega_e_hat*omega_e_hat + omega_e_dot_hat) * e3;   % Image 2
P1e_ddot = -(b3e_ddot*b3e' + 2*b3e_dot*b3e_dot' + b3e*b3e_ddot');
s_e_ddot = P1e_ddot*ref.b1_de + 2*P1e_dot*ref.b1_de_dot + P1e*ref.b1_de_ddot;
b1ec_ddot = (1/nse)*(P2e_dot*s_e_dot + P2e*s_e_ddot) ...
            - (b1ec'*s_e_dot)/nse * b1ec_dot;

% ---- commanded e3-axis rate and its derivative (Image 1 / Image 2) ------
omega_3ec     = b3e' * cross(b1ec, b1ec_dot);
omega_3ec_dot = b3e_dot' * cross(b1ec, b1ec_dot) ...
              + cross(b3e, b1ec)' * b1ec_ddot;

% ---- task-space errors ----
e_xE  = r_e - ref.r_ed;
e_RE3 = -b1ec' * b2e;                         % scalar orientation error
e_y   = [e_xE; e_RE3];                        % (4x1)

e_vE  = r_e_dot - ref.r_ed_dot;
e_wE3 = omega_3e - omega_3ec;                 % scalar
e_vy  = [e_vE; e_wE3];                        % (4x1)

yddot_d = [ref.r_ed_ddot; omega_3ec_dot];     % (4x1)

% ---- desired task inertia M_y (impedance_mode set in main_sim Section 1c) -
% M_y = [] is the 'natural' mode: resolve it to the current task inertia
% Lambda_y here (Lambda_y is state-dependent, only known now that dyn is
% unpacked).  M_y = Lambda_y gives Lambda_y*M_y^{-1} = I: plain impedance, and
% the GMO F_hat_y term below vanishes.  A non-empty M_y is the 'shaped' mode.
if isempty(M_y), M_y = Lambda_y; end          % [] -> natural inertia (no shaping)
Lam_Myinv = Lambda_y * (M_y \ eye(4));        % Lambda_y M_y^{-1}   (4x4)

% ---- estimated task-space disturbance force (Image "F_hat_y") ------------
% F_hat_y = (J_y^#)^T d_e_hat = J_1y d_t_hat + J_2y d_r_hat + J_3y d_rho_hat.
F_hat_y = J_1y*d_t_hat + J_2y*d_r_hat + J_3y*d_rho_hat;   % (4x1)

% ---- joint-torque control law ----
F_trans = u1*(R_0*e3) - m*g*e3;               % net translational force (3x1)
tau_rot = u2 - C_r*omega_0 - C_rp*rho;        % rotational part         (3x1)
stacked = [F_trans; tau_rot];                 % (6x1)

inner = [J_1y, J_2y]*stacked ...
        + Lambda_y*(J_y_dot*xi - yddot_d) ...
        + Lam_Myinv*(D_y*e_vy + K_y*e_y) ...
        - (Lam_Myinv - eye(4))*F_hat_y;        % (4x1)

% J_3y is FULL RANK by design: with joint 4 a roll about the EE z-axis, the
% EE-heading DOF gets its own independent joint column, so the arm-only task
% Jacobian is non-singular at the operating pose.  A plain linear solve thus
% suffices -- no damped-/pseudo-inverse needed.
u3 = -(J_3y \ inner) - C_rp'*omega_0 + C_p*rho;   % (nx1)

% ========================================================================
% Control -- Disturbance Observer (GMO): state derivative
% ========================================================================
% Formed AFTER u is known (it depends on u).  u is the TRANSFORMED wrench that
% drives the transformed dynamics:  u = [u1 R0 e3; u2; u3].
u_wrench = [u1*(R_0*e3); u2; u3];             % (6+n x1)
if gmo_on
    p_hat_dot = C_tilde'*xi - g_tilde + u_wrench + d_e_hat;   % (6+n x1)
else
    p_hat_dot = zeros(6+n, 1);
end

% ========================================================================
% Pack control outputs
% ========================================================================
ctrl.u1 = u1;
ctrl.u2 = u2;
ctrl.u3 = u3;
ctrl.u  = [u1; u2; u3];
% GMO observer: state derivative (for the augmented ODE) and estimates
ctrl.p_hat_dot = p_hat_dot;
ctrl.d_e_hat   = d_e_hat;
ctrl.d_t_hat   = d_t_hat;
ctrl.d_r_hat   = d_r_hat;
ctrl.d_rho_hat = d_rho_hat;
% handy intermediates (debugging / acceleration recovery)
ctrl.xi  = xi;
ctrl.f_d = f_d;
ctrl.R0c = R0c;
ctrl.e_y = e_y;
ctrl.e_vy = e_vy;
end
