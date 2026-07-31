function [Xdot, kin, tau, ctrl, dist] = closed_loop_dynamics(t, X, params)
%CLOSED_LOOP_DYNAMICS  Aerial-manipulator closed-loop dynamics.
%
%   [Xdot, kin, tau, ctrl, dist] = closed_loop_dynamics(t, X, params)
%   evaluates the state derivative of the aerial manipulator under
%   closed-loop control, following the architecture of Fig. 2:
%
%       X --[dynamics: re-coupling T]--> dyn (M_tilde,C_tilde,g_tilde, T,
%                                             M,C,g, J_y, ...)
%       dyn,ref --[controller]--------> ctrl (u1,u2,u3)            (transformed)
%       u --[de-coupling: tau = T'*u]-> tau                       (physical)
%       tau --[real dynamics M,C,g]---> Vdot --> Xdot
%
%   State X.  The physical state has length 18 + 2n:
%       X_plant = [ r_0(3); vec(R_0)(9); q(n); v_0(3); omega_0(3); qdot(n) ]
%   With the GMO disturbance observer active the state is AUGMENTED by the
%   observer momentum p_hat in R^{6+n}, appended at the tail:
%       X = [ X_plant(18+2n) ; p_hat(6+n) ]         (total 24 + 3n)
%   The plant indices are unchanged, so a plant-only X (no observer) also works
%   -- the observer is then simply disabled.
%
%   OPTIONAL OUTPUTS (for signal recording / post-processing)
%       tau  : physical generalized control  (length 6+n)
%              tau = [ base force(3); base moment(3); joint torques(n) ]
%       ctrl : full controller struct (u1, u2, u3, u, xi, R0c, e_y, plus the
%              GMO fields p_hat_dot, d_e_hat, d_t_hat, d_r_hat, d_rho_hat, ...)
%
%   These extra outputs let main_sim record exactly the quantities that were
%   used to drive the integration at each accepted step, without recomputing
%   the controller inside plot_result.
%
%       dist : struct describing the TRUE end-effector disturbance applied on
%              this call (fields .mode, .w_e, .Jt_we, .d_e, .d_t, .d_r, .d_rho).
%              This is ground truth for the plant + logging; the controller does
%              NOT see it (it estimates d_e via the GMO) -- so comparing dist.d_e
%              against ctrl.d_e_hat measures the observer's accuracy.
%
%   DISTURBANCE MODE (params.mode) -- the EE wrench w_e applied to the plant:
%       'nominal'      : no disturbance (w_e = 0).  The GMO estimate stays near
%                        zero (up to model/integration residuals).
%       'payload'      : constant hanging payload of mass params.payload_mass --
%                        weight -m_p*g*e3 (downward, inertial frame) at the EE.
%       'const_wrench' : constant arbitrary 6-D wrench params.ext_wrench_I
%                        ([fx fy fz mx my mz]) in the inertial frame (default
%                        2 N along +x).
%       'sine'         : zero-bias sine force params.sine_amp*sin(2*pi*
%                        params.sine_freq*t) along inertial +x (low frequency).
%   In every non-nominal mode the plant gets the physical term J_e^T w_e
%   (Image 1); the GMO estimates the transformed disturbance d_e_hat from the
%   momentum mismatch and the controller cancels d_t_hat in u1 and d_r_hat in
%   u2.  The arm channel is impedance-controlled: with the default task inertia
%   M_y = Lambda_y the arm disturbance is left to the Cartesian impedance
%   (D_y,K_y), so the EE settles at a small compliant offset.  (Setting a
%   shaped params.M_y reshapes the apparent inertia and activates the GMO
%   arm-force compensation term inside the controller.)
%
%   Requires on the path:  dynamics, plan_locked_trajectory, controller, hat.

% ========================================================================
% 0. Unpack the parts of the state needed below
% ========================================================================
n       = params.n;
R_0     = reshape(X(4:12), 3, 3);
v_0     = X(13+n:15+n);
omega_0 = X(16+n:18+n);
qdot    = X(19+n:18+2*n);
V       = [v_0; omega_0; qdot];
e3      = [0;0;1];

% ---- GMO observer state (optional state augmentation) -------------------
% The generalized-momentum observer carries its own state p_hat in R^{6+n}
% APPENDED to the plant state:  X = [ plant(18+2n) ; p_hat(6+n) ].  The plant
% indices above are unchanged (p_hat is at the tail), so every downstream
% unpack still works.  If X is plant-only (a standalone / sanity call), the
% observer is simply OFF.
n_plant = 18 + 2*n;                 % length of the physical state
n_obs   = 6 + n;                    % length of the observer momentum p_hat
have_obs = (numel(X) >= n_plant + n_obs);
if have_obs
    p_hat = X(n_plant+1 : n_plant+n_obs);
else
    p_hat = [];                     % -> controller runs with the GMO disabled
end

% ========================================================================
% 1. Dynamics (PDF1)  ->  dyn struct (+ kin for visualization)
%    Provides transformed (tilde) AND original (M,C,g) matrices, plus T.
% ========================================================================
[dyn, kin] = dynamics(X, params);

% ========================================================================
% 2. Reference trajectory evaluation  ->  ref struct
%    Planned ONCE in main_sim (params.ref_fcn); here we only EVALUATE it at
%    the current ODE time t.  Fallback: analytic EE-locked reference.
% ========================================================================
if isfield(params,'ref_fcn') && ~isempty(params.ref_fcn)
    ref = params.ref_fcn(t);                    % reference selected in main_sim
else
    ref = plan_locked_trajectory(t, params);    % fallback: EE-locked reference
end

% ========================================================================
% 3. End-effector disturbance (PDF2)
% ========================================================================
% ---- 3a. End-effector wrench w_e (expressed in {e}) ---------------------
% J_e maps V to the EE body twist in {e}, so the disturbance wrench w_e must be
% power-conjugate in {e}.  A wrench specified in the INERTIAL frame {I},
% w_I = [f_I; m_I], maps in as  w_e = [R_e'*f_I ; R_e'*m_I],  R_e = R_0*R_e^0.
%
% params.mode selects the disturbance (default 'nominal'):
%   'nominal'      : none (w_e = 0).
%   'payload'      : constant hanging payload, weight f_I = -m_p*g*e3 in {I}
%                    (pure force, zero moment) -> w_e time-varying in {e}.
%   'const_wrench' : constant arbitrary 6-D wrench params.ext_wrench_I in {I}
%                    ([fx fy fz mx my mz]; default = 2 N along +x).
%   'sine'         : zero-bias sine force A*sin(2*pi*fr*t) along +x in {I}
%                    (A = params.sine_amp, fr = params.sine_freq; low freq).
mode = 'nominal';
if isfield(params, 'mode') && ~isempty(params.mode), mode = params.mode; end

R_e = R_0 * dyn.R_e_0;             % EE orientation in {I} (maps {I}->{e} wrenches)
w_e = zeros(6,1);                  % EE wrench in {e}: [force(3); moment(3)]
switch lower(mode)
    case 'nominal'
        % w_e = 0  (no disturbance)

    case 'payload'
        m_p = 0.200;                                     % default payload (kg)
        if isfield(params,'payload_mass') && ~isempty(params.payload_mass)
            m_p = params.payload_mass;
        end
        f_I = -m_p * params.g * e3;                      % constant weight in {I}
        w_e = [R_e'*f_I; zeros(3,1)];                    % -> {e}

    case 'const_wrench'
        w_I = [2;0;0; 0;0;0];                            % default: 2 N along +x
        if isfield(params,'ext_wrench_I') && ~isempty(params.ext_wrench_I)
            w_I = params.ext_wrench_I(:);
        end
        w_e = [R_e'*w_I(1:3); R_e'*w_I(4:6)];            % inertial wrench -> {e}

    case 'sine'
        A  = 2.0;   fr = 0.5;                            % default: 2 N, 0.5 Hz (low)
        if isfield(params,'sine_amp')  && ~isempty(params.sine_amp),  A  = params.sine_amp;  end
        if isfield(params,'sine_freq') && ~isempty(params.sine_freq), fr = params.sine_freq; end
        f_I = [A*sin(2*pi*fr*t); 0; 0];                  % zero-bias sine in +x, {I}
        w_e = [R_e'*f_I; zeros(3,1)];                    % -> {e}

    otherwise
        error('closed_loop_dynamics:mode', ...
              ['Unknown params.mode "%s" ' ...
               '(use nominal | payload | const_wrench | sine).'], mode);
end

% ---- 3b. Physical + transformed disturbance -----------------------------
% Physical generalized disturbance force:  J_e^T w_e          (Image 1)
% Transformed disturbance (what the estimator delivers to the controller):
%   d_e = T^{-T} J_e^T w_e = (T')^{-1} (J_e^T w_e) = dyn.T' \ (J_e^T w_e)  (Image 4)
Jt_we = dyn.J_e' * w_e;            % (6+n x 1)  physical generalized disturbance
d_e   = dyn.T' \ Jt_we;           % (6+n x 1)  transformed disturbance d_e

dist.mode  = mode;
dist.w_e   = w_e;
dist.Jt_we = Jt_we;
dist.d_e   = d_e;
dist.d_t   = d_e(1:3);            % translational part  -> compensated in u1
dist.d_r   = d_e(4:6);            % rotational part     -> compensated in u2
dist.d_rho = d_e(7:6+n);          % arm part            -> LEFT to the impedance

% ========================================================================
% 4. Controller (transformed control inputs) -- WITH the GMO observer
% ========================================================================
% The controller no longer receives the true disturbance: it ESTIMATES d_e
% online from the momentum mismatch (p - p_hat) via the generalized-momentum
% observer (GMO).  It is handed the observer state p_hat and returns its
% derivative ctrl.p_hat_dot, which is appended to Xdot below.  (`dist`, the
% ground-truth transformed disturbance, is still computed above for the plant
% and for logging / validation of the estimate -- but it is NOT fed to the
% controller.)
ctrl = controller(t, X, dyn, ref, params, [], p_hat);

% ========================================================================
% 5. De-coupling transformation  u -> tau,  then real-dynamics integration
% ========================================================================
% Transformed wrench (note u1 enters as the force vector u1*R0*e3, per eq. 81):
u_wrench = [ ctrl.u1 * (R_0*e3) ;   % translational force (3x1)
             ctrl.u2            ;   % base moment         (3x1)
             ctrl.u3            ];  % joint torques       (n x1)

% De-coupling:  u = T^{-T} tau   =>   tau = T^T u   (eq. 79)
tau = dyn.T' * u_wrench;            % physical generalized control (6+n x1)

% Real (original) dynamics WITH the end-effector disturbance (Image 1):
%   M Vdot + C V + g = tau + J_e^T w_e        (J_e^T w_e = 0 in 'nominal')
Vdot = dyn.M \ (tau + Jt_we - dyn.C * V - dyn.g);

v_0_dot     = Vdot(1:3);
omega_0_dot = Vdot(4:6);
qddot       = Vdot(7:6+n);

% ========================================================================
% 6. Kinematic differential equations -- eqs. (3),(4) and q_dot
% ========================================================================
r_0_dot = R_0 * v_0;
R_0_dot = R_0 * hat(omega_0);

Xdot = [ r_0_dot;
         R_0_dot(:);
         qdot;
         v_0_dot;
         omega_0_dot;
         qddot ];

% ---- Append the GMO observer dynamics (only if the state was augmented) --
% p_hat_dot = C_tilde^T xi - g_tilde + u + d_e_hat   (formed inside controller,
% after u).  Keeping it in the augmented ODE lets ode45 integrate the observer
% consistently with the plant (a persistent-variable Euler step would be
% corrupted by the solver's rejected steps and multi-stage evaluations).
if have_obs
    Xdot = [Xdot; ctrl.p_hat_dot];
end
end