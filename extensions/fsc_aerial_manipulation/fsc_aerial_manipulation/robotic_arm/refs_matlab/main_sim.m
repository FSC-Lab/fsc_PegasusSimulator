%MAIN_SIM  Initialization + closed-loop simulation for the aerial manipulator.
%
%   Pipeline (each stage in its own file):
%     dynamics(X,params)              -> dyn  (M_tilde,C_tilde,g_tilde, M,C,g, T, J_y, ...)
%     <reference>(t,params)           -> ref  (selected below, see Section 3b)
%     controller(t,X,dyn,ref,params)  -> ctrl (u1,u2,u3)
%     closed_loop_dynamics(t,X,params)-> Xdot (de-coupling tau=T'u + real dynamics)
%
%   REFERENCE SELECTION (params.ref_mode, set in Section 1b):
%     'hover'|'line'|'sine'|'circle'|'poly'
%                    -> plan_locked_trajectory: the end-effector is LOCKED to
%                       the platform, so the arm holds its initial (non-singular)
%                       joint configuration.  X0 is the initial state built in
%                       Section 2; the rest-to-rest reference starts there, so
%                       the initial tracking error is zero.
%     'compatible'   -> plan_compatible_trajectory: an OFFLINE-planned,
%                       dynamically-compatible CoM trajectory where the ARM MOVES.
%                       It OVERRIDES X0 with the planned t=0 configuration (zero
%                       initial error) and provides the simulation horizon.
%
%   Required on the path:
%     dynamics.m, plan_locked_trajectory.m, plan_compatible_trajectory.m,
%     controller.m, closed_loop_dynamics.m, hat.m,
%     visualize_aerial_manipulator.m, plot_result.m, visualize_reference.m,
%     generate_video.m
%
%   Naming follows the LaTeX symbols in the flowchart, e.g.
%     r_0 -> r_0,  R_0 -> R_0,  r_0c_0 -> r_{0c}^0,  h_i_im1 -> h_i^{i-1}.

clear; close all; clc;

% ---- Put all helper folders on the path ---------------------------------
%   Every function lives under ./utils (utils_control, utils_dynamics,
%   utils_planning, utils_plot, utils_singularity).  Add them recursively,
%   relative to THIS script's location so it works from any current folder.
this_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(this_dir, 'utils')));

% ========================================================================
% 1.  GEOMETRY / INERTIAL PARAMETERS
% ========================================================================
n = 4;                         % OpenMANIPULATOR-X has 4 revolute joints

% ---- Quadrotor (Holybro X650) -------------------------------------------
m_0  = 2.47607955;                             % kg (Holybro X650; from CAD postprocessor)
I_0  = [ 0.06301228,  0.00000185,  0.00000234;   % kg·m² full body inertia tensor {0}
         0.00000185,  0.06334175, -0.00005420;   %   (CAD postprocessor; CoM offset omitted)
         0.00000234, -0.00005420,  0.09868092];

% ---- OpenMANIPULATOR-X links --------------------------------------------
L = [0.077, 0.128, 0.124, 0.126];              % link lengths (m)  (l2: 0.130 -> 0.128)
m_link = [0.10483, 0.14235, 0.13467, 0.23551]; % link masses (kg)  (OM-X links 1..4)

% Joint-axis configuration:
%   joint 1 about z_0  -> h_1^0 = [0;0;1]
%   joints 2,3,4 about y_{i-1} -> h_i^{i-1} = [0;1;0]
h_i_im1 = zeros(3, n);
h_i_im1(:,1) = [0; 0; 1];
h_i_im1(:,2) = [0; 1; 0];
h_i_im1(:,3) = [0; 1; 0];
h_i_im1(:,4) = [0; 0; 1]; % rotate z

% Link offsets l_i: O_{i-1} -> O_i expressed in {i}.
delta2 = 0.024;                  % link-2 in-plane forward bend (+x_2) offset (m)
l_i = zeros(3, n);
l_i(:,1) = [0;      0; -L(1)];   % O_0 -> O_1 : along -z_1
l_i(:,2) = [delta2; 0; -L(2)];   % O_1 -> O_2 : BENT -- forward +x_2 AND down -z_2
l_i(:,3) = [L(3);   0;  0   ];   % O_2 -> O_3 : along +x_3
l_i(:,4) = [0;    0; -L(4)]; % work 1: extend z (either +/- z)

% Body-frame link inertias I_i^i.  TWO contributions:
%
%   (1) LINK inertia -- official ROBOTIS CAD tensor per OpenMANIPULATOR-X link,
%       "Inertia Tensor at CENTER OF GRAVITY" from
%         https://docs.robotis.com/docs/systems/openmanipulator_x/specification
%       Given in [g*mm^2] in ROBOTIS's link frame (z = link long axis).  Two
%       fixed steps: units g*mm^2 -> kg*m^2 (x 1e-9), and rotate that frame into
%       our {i} via R_r2o (built with joint_rotation, see below):
%         I_i^i = 1e-9 * R_r2o * I_links * R_r2o'.
%       (Tensor is about the TRUE CoG; dynamics.m keeps the CoM at l_i/2, so no
%       parallel-axis shift is applied.)
%
%   (2) Reflected ROTOR inertia of the geared DYNAMIXELs, J_arm = N_gear^2*J_rotor,
%       added rank-1 along each spin axis h_i^{i-1}.  NECESSARY for the real
%       geared motor: it is O(1e-2) kg*m^2 -- far larger than any link inertia --
%       so omitting it leaves joint 4 nearly massless about its own axis and the
%       joint accelerations explode.  Flag add_motor_inertia toggles it.
% ------------------------------------------------------------------------
add_motor_inertia = true;        % MODE FLAG -- TUNE:
                                 %   true  : add reflected rotor inertia J_arm to each joint
                                 %   false : ROBOTIS link inertia only (no motor armature)
params.add_motor_inertia = add_motor_inertia;   % record the mode

N_gear  = 353.5;                 % XM430-W350 gear ratio
J_rotor = 1.6e-7;                % kg*m^2, motor-side rotor inertia
                                 %   -> identify precisely from datasheet / swing test;
                                 %      1.6e-7 gives J_arm ~ 2.0e-2, validated stable.
J_arm   = N_gear^2 * J_rotor;    % ~2.0e-2 kg*m^2 reflected armature inertia per joint

% ---- ROBOTIS CAD link inertias at CoG [g*mm^2], in ROBOTIS link frame -----
I_links = zeros(3,3,n);
I_links(:,:,1) = [   3.3802078e4,  0.0,          0.0        ;   % link 1
                     0.0,          2.9569411e4,  2.2121514e2;
                     0.0,          2.2121514e2,  1.7610252e4];
I_links(:,:,2) = [   2.3711450e5, -2.7513999e2,  2.6666982e4;   % link 2
                    -2.7513999e2,  2.4488355e5,  1.3126636e3;
                     2.6666982e4,  1.3126636e3,  4.2967059e4];
I_links(:,:,3) = [   1.8688812e5,  0.0,          1.8290300e0;   % link 3
                     0.0,          1.7818139e5,  1.2221090e3;
                     1.8290300e0,  1.2221090e3,  2.4809204e4];
I_links(:,:,4) = [   2.6708326e5,  2.2286943e2,  2.2615865e4;   % link 4
                     2.2286943e2,  2.0769766e5,  1.5305492e1;
                     2.2615865e4,  1.5305492e1,  1.7393236e5];

% ---- ROBOTIS-link-i frame -> our {i} frame --------------------------------
%   Built with joint_rotation([0;1;0],q) = Ry(q) so the axis/angle is explicit
%   and needs no hand-checking (their & our y coincide -> pure y-rotations).
R_r2o = zeros(3,3,n);
R_r2o(:,:,1) = joint_rotation([0;1;0],  pi  );   % links 1,2,4: hang along -z_0
R_r2o(:,:,2) = joint_rotation([0;1;0],  pi  );
R_r2o(:,:,3) = joint_rotation([0;1;0], -pi/2);   % link 3: extends along +x_0
R_r2o(:,:,4) = joint_rotation([0;1;0],  pi  );

g_mm2_to_kg_m2 = 1e-9;           % g*mm^2 -> kg*m^2

I_i_i = zeros(3,3,n+1);
I_i_i(:,:,1) = I_0;
for i = 1:n
    R  = R_r2o(:,:,i);
    Ii = g_mm2_to_kg_m2 * (R * I_links(:,:,i) * R.');   % (1) CAD link inertia in {i}
    if add_motor_inertia
        spin = h_i_im1(:,i);                              % this joint's spin axis in {i}
        Ii   = Ii + J_arm * (spin * spin.');              % (2) reflected rotor inertia
    end
    I_i_i(:,:,i+1) = Ii;
end

% ---- Pack params --------------------------------------------------------
params.n       = n;
params.m_i     = [m_0, m_link];        % 1 x (n+1)
params.L       = L;
params.l_i     = l_i;
params.h_i_im1 = h_i_im1;
params.I_i_i   = I_i_i;
params.g       = 9.81;

% ========================================================================
% 1b.  REFERENCE TRAJECTORY + SIMULATION / DISTURBANCE MODE
% ========================================================================
% ---- Reference selection (see the header) -------------------------------
%   locked shapes -> plan_locked_trajectory (EE fixed to the platform):
%       'hover' | 'line' | 'sine' | 'circle' | 'poly'
%       (per-shape tuning lives in plan_locked_trajectory.m -> traj_config)
%   planned        -> plan_compatible_trajectory (arm moves; grasp task tuned
%       in Section 1 of plan_compatible_trajectory.m):
%       'compatible'
params.ref_mode = 'hover';           % <-- SELECT THE REFERENCE HERE   -- TUNE

%   End-effector DISTURBANCE mode (all wrenches applied AT the EE point):
%     'nominal'      : no disturbance.
%     'payload'      : constant hanging payload -- weight -m_p*g*e3 (inertial
%                      frame), a pure downward force.
%     'const_wrench' : constant arbitrary 6-D wrench params.ext_wrench_I in the
%                      inertial frame (default: a force in +x).
%     'sine'         : zero-bias sine force along inertial +x, low frequency.
%   In every non-nominal mode the GMO (params.use_gmo, below) ESTIMATES the
%   disturbance online and the controller cancels its estimate in translation
%   (u1) and rotation (u2); the arm channel (u3) is impedance control, so the
%   EE settles at a small compliant offset.  The reference trajectory is
%   UNCHANGED between modes: the impedance loop adjusts the EE pose.
params.mode = 'sine';    % 'nominal' | 'payload' | 'const_wrench' | 'sine'  -- TUNE

% ---- Disturbance parameters (each used only by its mode) ----------------
params.payload_mass = 0.200;             % 'payload'     : hanging mass (kg) -- 200 g
params.ext_wrench_I = [1;0;0; 0;0;0];    % 'const_wrench' : [fx fy fz mx my mz] in {I} (N, N*m)
params.sine_amp     = 0.25;               % 'sine'        : force amplitude along +x in {I} (N)
params.sine_freq    = 0.25;               % 'sine'        : frequency (Hz) -- keep LOW

% ---- GMO disturbance observer: master on/off switch ---------------------
% true  : run the generalized-momentum observer.  The state is augmented with
%         the observer momentum p_hat (Section 3d) and the controller cancels
%         the ESTIMATED d_t / d_r in u1 / u2 (and shapes the arm channel if the
%         'shaped' impedance mode is selected in Section 1c).
% false : GMO OFF -> pure nominal controller, NO disturbance compensation.
%         The state is left un-augmented, so controller runs with d_e_hat = 0;
%         in a disturbance mode the Cartesian impedance alone absorbs the load.
%         (Use this as the GMO-vs-no-GMO A/B baseline.)
params.use_gmo = true;               % true | false                -- TUNE

% ========================================================================
% 1c.  CONTROLLER GAINS  (single place to tune -- controller.m only READS these)
% ========================================================================
% Every controller gain is defined here as a params.* field; controller.m keeps
% no defaults of its own.  Grouped by what they shape.

% ---- Rigid-body tracking gains (translation + attitude) -----------------
params.k_x   = 10;           % CoM position gain      (scalar, acts on 3-vectors)
params.k_v   = 5;            % CoM velocity gain
params.k_R   = 8;            % attitude gain
params.k_w   = 2;            % angular-rate gain
params.M_r_d = eye(3);       % desired rotational inertia (3x3) -- design choice

% ---- Task-space impedance: damping D_y, stiffness K_y (4x4) -------------
% 4x4 = [x;y;z] EE position + EE-heading.  With the reflected rotor inertia now
% in the model, FULL heading gain is stable (EE-yaw control), validated in
% +x/+y/+z/+xyz and 30-60 deg yaw slews.
%
% Stiffness vs. load: the steady-state EE deflection under an uncompensated
% task force f is ~ K_y^{-1} f.  A very soft K_y (e.g. 4 N/m) needs a ~0.5 m sag
% to hold a 2 N load -- unreachable -> arm collapse.  So in EVERY disturbance
% mode K_y is AUTO-SIZED from a representative disturbance-force magnitude F_dist
% for a small, reachable sag (params.ee_sag_target):
%     K_y = F_dist/sag,   D_y = 2*sqrt(K_y)   (~critical; same damping ratio at
% any load, since an external force does not change the task inertia).
%   F_dist = m_p*g (payload) | ||force|| (const_wrench) | amplitude (sine).
params.ee_sag_target = 0.01;         % desired EE sag under the load (m) -- TUNE
switch lower(params.mode)
    case 'nominal',      F_dist = 0;
    case 'payload',      F_dist = params.payload_mass * params.g;   % payload weight (N)
    case 'const_wrench', F_dist = norm(params.ext_wrench_I(1:3));   % force magnitude (N)
    case 'sine',         F_dist = params.sine_amp;                  % peak force (N)
    otherwise
        error('main_sim:mode', ...
              'Unknown params.mode "%s" (use nominal|payload|const_wrench|sine).', ...
              params.mode);
end
if F_dist > 0
    k_pos      = F_dist / params.ee_sag_target;         % position stiffness (N/m)
    params.K_y = k_pos         * eye(4);                % task stiffness
    params.D_y = 2*sqrt(k_pos) * eye(4);                % task damping (~critical)
else
    params.K_y = 4 * eye(4);                            % soft nominal stiffness
    params.D_y = 4 * eye(4);                            % soft nominal damping
end

% ---- Task-space desired inertia M_y: NATURAL vs SHAPED ------------------
% 'natural' : M_y = Lambda_y (the robot's own task inertia) -> NO inertia
%             shaping; the arm law is plain impedance and the GMO arm-force term
%             vanishes.  Reproduces the previous validated behaviour.  Signalled
%             by params.M_y = [] (Lambda_y is state-dependent, so controller.m
%             resolves it online).
% 'shaped'  : impose a DESIGN task inertia params.M_y ~= Lambda_y (4x4 SPD,
%             translational block isotropic).  Realizing a different apparent
%             inertia REQUIRES the estimated task force F_hat_y, so this mode is
%             meaningful only with params.use_gmo = true.
params.impedance_mode = 'shaped';   % 'natural' | 'shaped'         -- TUNE
switch lower(params.impedance_mode)
    case 'natural'
        params.M_y = [];                             % -> Lambda_y (resolved in controller)
    case 'shaped'
        m_y_t = 1.0;                                 % desired translational inertia (kg)
        m_y_r = 0.05;                                % desired EE-heading inertia (kg m^2)
        params.M_y = diag([m_y_t m_y_t m_y_t m_y_r]);% isotropic translation block
    otherwise
        error('main_sim:impedance_mode', ...
              'Unknown params.impedance_mode "%s" (use natural|shaped).', ...
              params.impedance_mode);
end

% ---- GMO disturbance-observer gain K_o (6+n) ---------------------------
% Per-channel estimation bandwidth (rad/s); the estimate is a first-order
% low-pass of the true disturbance: d_e_hat_dot = K_o (d_e - d_e_hat).
k_o_t = 20;  k_o_r = 20;  k_o_rho = 20;              % -- TUNE
params.K_o = blkdiag(k_o_t*eye(3), k_o_r*eye(3), k_o_rho*eye(n));

% ---- Run banner ---------------------------------------------------------
fprintf('Reference mode : %s\n', upper(params.ref_mode));
fprintf('Simulation mode: %s', upper(params.mode));
switch lower(params.mode)
    case 'payload'
        fprintf('  (payload = %.0f g)', 1e3*params.payload_mass);
    case 'const_wrench'
        fprintf('  (const wrench {I}: f=[%.1f %.1f %.1f] N, m=[%.2f %.2f %.2f] N*m)', ...
                params.ext_wrench_I);
    case 'sine'
        fprintf('  (sine force +x {I}: %.1f N @ %.2f Hz)', params.sine_amp, params.sine_freq);
end
fprintf('\n');
if ~strcmpi(params.mode, 'nominal')
    fprintf('  impedance auto-sized: target sag = %.0f mm -> K_y = %.1f, D_y = %.1f\n', ...
            1e3*params.ee_sag_target, params.K_y(1,1), params.D_y(1,1));
end
gmo_flags = {'OFF (no disturbance compensation)', 'ON'};
fprintf('GMO observer   : %s\n',   gmo_flags{1 + logical(params.use_gmo)});
fprintf('Impedance mode : %s\n\n', upper(params.impedance_mode));

% ========================================================================
% 2.  INITIAL STATE  X = [r_0; vec(R_0); q; v_0; omega_0; qdot]
% ========================================================================
r_0_init = [0; 0; 0.5];                              % quadrotor 1 m up

% Initial attitude.  The controller's thrust acts along +R_0*e3, and the
% hover force balance needs R_0*e3 aligned with the commanded thrust
% b3c = f_d/||f_d|| ~= +e3 ("up").  So the drone must start UPRIGHT.
% A z-down init (R_0*e3 = -e3) sits on the geometric controller's UNSTABLE
% ~180 deg equilibrium and diverges within ~0.1 s.  (Re-applied here because
% the uploaded file had reverted to z-down.)
R_attitude_init = eye(3);          % set a yaw/tilt here if you want one
R_0_init        = R_attitude_init; % upright: third column = [0;0;1] = +e3

% Non-singular pose for the NEW axis layout (joint 4 = ROLL about the EE
% z-axis, link 4 along z so the EE sits ON the roll axis).
%   * q2 + q3 = 0  -> EE z-axis stays vertical (b3e = e3).
%   * q4 = 0       -> zero initial wrist roll, so R_e^0 = I, i.e. the
%                     end-effector frame is aligned with the drone body frame.
% Joints 1,2,3 span the 3-D EE position and joint 4 drives the EE heading
% directly, so the arm-only task Jacobian J_3y is FULL RANK here
% (numerically cond(J_3y-block) ~= 90 -- no inversion tricks needed).
q_init_deg   = [0; 0; -20; 0];        % assign in DEGREES (intuitive)  -- TUNE
q_init       = deg2rad(q_init_deg); % -> rad for the dynamics
v_0_init     = zeros(3,1);
omega_0_init = zeros(3,1);
qdot_init    = zeros(n,1);

X0 = [ r_0_init;
       R_0_init(:);
       q_init;
       v_0_init;
       omega_0_init;
       qdot_init ];

% ========================================================================
% 3.  PRELIMINARY SETUP  (initial kinematics, hover anchors, reference)
%     Everything the formal run needs, computed ONCE and offline.  On exit:
%       params.ref_fcn  : @(t) -> full controller reference struct
%       params.traj_tag : short label for saved figure/video filenames
%       X0              : initial state ON the reference (zero initial error)
%       T_sim           : simulation horizon (s)
% ========================================================================
t0 = 0;

% --- 3a. Kinematics of the initial configuration (feeds the hover anchors) ---
[~, kin] = dynamics(X0, params);

% --- 3b. Hover setpoints derived from the initial state (compatible by
%     construction: x_cd and r_ed share the same r_0, R_0).  Expressed in {I}. ---
R0_0 = reshape(X0(4:12), 3, 3);
params.hover.x_cd  = X0(1:3) + R0_0 * kin.r_0c_0;          % initial system CoM
params.hover.r_ed  = X0(1:3) + R0_0 * kin.r_0e_0;          % initial end-effector
params.hover.b1_d  = R0_0 * [1;0;0];                       % initial base heading  (x_0 in {I})
params.hover.b1_de = (R0_0 * kin.R_e_0) * [1;0;0];         % initial EE heading    (x_e in {I})

% --- 3c. SELECT + BUILD the reference trajectory (see Section 1b) ---------
switch lower(params.ref_mode)
    case 'compatible'
        % ---- v2: dynamically-compatible planned CoM trajectory (arm MOVES) ----
        %   Configure the grasp (EE start/goal, approach tilt, yaw slew) in
        %   SECTION 1 of plan_compatible_trajectory.m.  Defaults: EE start =
        %   params.hover.r_ed (set above).  Pass struct('plot',false) to suppress
        %   its diagnostic figure.
        plan            = plan_compatible_trajectory(params);   % offline solve, runs ONCE
        params.ref_fcn  = plan.ref;            % closed_loop_dynamics reads this
        params.traj_tag = 'compatible';
        X0              = plan.X0;   % start ON the plan: r_0, R_0 AND joint angles q0 are
                                     % the planned t=0 configuration -> zero init error.
        T_sim           = plan.T;    % horizon from the offline planner
        % J_3y^0 conditioning is evaluated over the ENTIRE planned trajectory.
        fprintf(['Planner: dyn defect %.2e m | beta [%.0f,%.0f] deg\n' ...
                 '  J_3y^0 (geom) over trajectory: cond start %.0f / max %.0f | sigma_min start %.3e / min %.3e\n'], ...
                plan.diag.max_dyn_defect, plan.diag.beta_range_deg, ...
                plan.diag.cond_J3y_t0, plan.diag.max_cond_J3y, ...
                plan.diag.sigma_J3y_t0, plan.diag.min_sigma_J3y);

        % GUARD: the controller inverts the arm task Jacobian (u3 = J_3y \ inner).
        % A near-singular PLANNED configuration makes u3 explode and ode45 fails at
        % t~0.  Stop here -- with a clear message -- instead of feeding a singular
        % start to the integrator.  Fix in plan_compatible_trajectory.m Section 1.
        if plan.diag.max_cond_J3y > 100
            error(['Planned arm config near-singular (max cond(J_3y^0)=%.0f). Adjust ' ...
                   'tp.beta0 (aim 40-70 deg) or tp.split (0.25/0.75) in the planner.'], ...
                  plan.diag.max_cond_J3y);
        end

    case {'hover','line','sine','circle','poly'}
        % ---- v1: EE LOCKED to the platform (arm holds its configuration) ------
        %   The locked shape is params.ref_mode ('hover'|'line'|'sine'|'circle'|
        %   'poly'); per-shape tuning lives in plan_locked_trajectory.m.  X0 stays
        %   as the initial state built in Section 2, and the rest-to-rest reference
        %   starts there, so the initial tracking error is zero.
        params.traj_type = params.ref_mode;                        % locked-shape selector
        params.ref_fcn   = @(tq) plan_locked_trajectory(tq, params);
        cfg_locked       = plan_locked_trajectory('config', params);
        params.traj_tag  = cfg_locked.type;
        T_sim            = cfg_locked.T;                            % type-dependent duration

        % J_3y^0 conditioning over the ENTIRE trajectory: locked mode holds q at
        % q_init, and J_3y^0 depends only on q (invariant to base pose), so cond /
        % sigma_min are CONSTANT along the trajectory -> evaluate once at q_init.
        s_lk = svd(arm_task_jacobian(X0(13:12+n), params));
        fprintf(['Locked (%s): arm holds q0 -> J_3y^0 (geom) constant over trajectory\n' ...
                 '  cond(J_3y^0) = %.0f | sigma_min = %.3e\n'], ...
                params.traj_tag, s_lk(1)/s_lk(end), s_lk(end));

    otherwise
        error('main_sim:ref_mode', ...
              'Unknown params.ref_mode "%s" (use compatible|hover|line|sine|circle|poly).', ...
              params.ref_mode);
end

% ========================================================================
% 3d.  GMO DISTURBANCE OBSERVER -- augment the state with p_hat(0)
% ========================================================================
% The generalized-momentum observer (GMO) has its own state p_hat in R^{6+n}
% (an estimate of the transformed momentum p = M_tilde*xi).  It is carried by
% the ODE state, APPENDED after the physical state:
%       X = [ X_plant(18+2n) ; p_hat(6+n) ]     (total 24 + 3n).
% Initialize p_hat(0) = M_tilde(0) xi(0) = p(0) so the initial estimate is
% exact, d_e_hat(0) = K_o (p - p_hat) = 0 -> no start-up transient (and the
% zero-initial-error property below is preserved).  This runs AFTER X0 is final
% (Section 3c may have overridden it with the planned start).
% GATED by params.use_gmo: when OFF, X0 is left un-augmented, so the whole
% downstream chain (closed_loop_dynamics -> controller) detects a plant-only
% state and runs with d_e_hat = 0 (no disturbance compensation).
if params.use_gmo
    [dyn_init, ~] = dynamics(X0, params);
    V0     = X0(13+n:18+2*n);                % [v_0; omega_0; qdot] at t = 0
    xi0    = dyn_init.T * V0;                 % transformed velocity
    p_hat0 = dyn_init.M_tilde * xi0;          % transformed momentum p(0)
    X0     = [X0; p_hat0];                    % augmented initial state
end

% ========================================================================
% 4.  QUICK SANITY CHECK  (single-shot evaluation before the formal run)
% ========================================================================
% --- Single-shot closed-loop call at the SELECTED reference; refreshes kin at
%     the final X0 (planned start in 'compatible' mode) and exposes ctrl. -----
[~, kin, ~, ctrl0] = closed_loop_dynamics(t0, X0, params);

% --- Initial whole-system tracking errors (should all be ~0 at a compatible
%     start): system CoM position, platform orientation, EE position, EE yaw.
%     Same definitions as plot_result Figure 1. -------------------------------
R_0  = reshape(X0(4:12), 3, 3);
ref0 = params.ref_fcn(t0);                               % selected reference
vee  = @(S) [S(3,2); S(1,3); S(2,1)];                    % so(3) -> R^3
e_x   = (X0(1:3) + R_0*kin.r_0c_0) - ref0.x_cd;          % system CoM position
e_R   = 0.5 * vee(ctrl0.R0c.'*R_0 - R_0.'*ctrl0.R0c);    % platform orientation
e_xE  = ctrl0.e_y(1:3);                                  % end-effector position
e_yaw = ctrl0.e_y(4);                                    % end-effector yaw

fprintf('=== Initial tracking errors (should be ~0) ===\n');
fprintf('||e_x||   = %.3e   (system CoM position)\n',   norm(e_x));
fprintf('||e_R||   = %.3e   (platform orientation)\n',  norm(e_R));
fprintf('||e_xE||  = %.3e   (end-effector position)\n', norm(e_xE));
fprintf('|e_yaw|   = %.3e   (end-effector yaw)\n',      abs(e_yaw));

% --- dyn.J_3y conditioning AT t=0 ONLY.  This is the block the controller
%     actually inverts (u3 = J_3y \ inner) -- inertia-weighted, so distinct
%     from the geometric J_3y^0.  Evaluated only at t=0 (a trajectory-wide pass
%     needs a full dynamics() solve per sample); the geometric J_3y^0 over the
%     ENTIRE trajectory is reported in Section 3c above. ----------------------
[dyn0, ~] = dynamics(X0, params);
sd0 = svd(dyn0.J_3y);                                  % matrix the controller inverts
fprintf('dyn.J_3y (controller inverts this) AT t=0: cond %.1f | sigma_min %.3e\n', ...
        sd0(1)/sd0(end), sd0(end));

fprintf('Configuration computed successfully.\n\n');

% ========================================================================
% 5.  Initial-configuration + reference visualization
% ========================================================================
% visualize_aerial_manipulator(kin, params, 'Title', ...
%     sprintf('Initial Configuration (%s mode)', params.mode));

% Reference sanity plots (locked only; the compatible planner draws its own).
% if ~strcmpi(params.ref_mode, 'compatible')
%     visualize_reference(params);
% end

% ========================================================================
% 6.  Time-domain closed-loop simulation  (ode45, fixed-length stepping)
% ========================================================================
%   ode45 is adaptive by nature.  To make it step at a fixed length we
%     (a) request output on a uniform grid  t0:dt:tf, and
%     (b) cap the internal step with 'MaxStep' = dt  (and InitialStep = dt).
%   Relaxed tolerances stop it from subdividing far below dt, which is what
%   was driving the long run time.  (A *truly* fixed step would need a custom
%   RK loop; MaxStep is the closest ode45 setting.)  If the result looks
%   noisy or diverges, DECREASE dt; if it is fine but slow, increase dt.
dt       = 0.01;                                    % fixed step length (s) -- TUNE
t_span   = [0, T_sim];                               % horizon from the selected reference
t_grid   = (t_span(1):dt:t_span(2)).';               % uniform output grid
ode_opts = odeset('MaxStep', dt, 'InitialStep', dt, ...
                  'RelTol', 1e-4, 'AbsTol', 1e-6);
odefun   = @(t, X) closed_loop_dynamics(t, X, params);

fprintf('Running ode45 (fixed-length dt = %.4f s) over [%.1f, %.1f] s ...\n', ...
        dt, t_span(1), t_span(2));
[t_hist, X_hist] = ode45(odefun, t_grid, X0, ode_opts);
X_hist = X_hist.';                                   % columns = time steps
fprintf('Simulation finished: %d steps.\n\n', numel(t_hist));

% ========================================================================
% 6b. RECORD signals at the accepted steps  (states + physical control tau)
% ========================================================================
%   ode45 calls closed_loop_dynamics several times per accepted step (the RK
%   stages), so logging *inside* the ODE would mix stage evaluations and
%   produce non-monotonic / duplicated samples.  Instead we re-evaluate the
%   closed loop at the accepted (t_hist, X_hist) points: because the control
%   law is a deterministic function of (t, X), this reproduces *exactly* the
%   tau that drove the integration at each output time -- i.e. a faithful
%   recording of the realized signals.
N = numel(t_hist);
vee = @(S) [S(3,2); S(1,3); S(2,1)];      % so(3) -> R^3

log.t    = t_hist(:).';                   % 1 x N   time stamps
log.X    = X_hist;                        % (18+2n) x N   recorded states
log.tau  = zeros(6+n, N);                 % physical control [F0(3); M0(3); tau_q(n)]
log.u    = zeros(4+n, N);                 % transformed control [u1; u2(3); u3(n)]
log.r0   = zeros(3, N);                   % base position (inertial)
log.rc   = zeros(3, N);                   % system CoM     (inertial)
log.re   = zeros(3, N);                   % end-effector   (inertial)
log.e_x  = zeros(3, N);                   % CoM position error
log.e_R  = zeros(3, N);                   % base attitude error
log.e_xE = zeros(3, N);                   % EE position error
log.e_yaw= zeros(1, N);                   % EE yaw error
log.d_t  = zeros(3, N);                   % TRUE transformed disturbance: translation
log.d_r  = zeros(3, N);                   % TRUE transformed disturbance: rotation
log.d_rho= zeros(n, N);                   % TRUE transformed disturbance: arm
log.d_t_hat   = zeros(3, N);              % GMO ESTIMATE: translation  (-> u1)
log.d_r_hat   = zeros(3, N);              % GMO ESTIMATE: rotation     (-> u2)
log.d_rho_hat = zeros(n, N);              % GMO ESTIMATE: arm          (-> u3 via F_hat_y)
log.w_e  = zeros(6, N);                   % EE wrench in {e} [force(3); moment(3)]

fprintf('Recording states + control tau over %d steps ...\n', N);
for k = 1:N
    tk = t_hist(k);
    Xk = X_hist(:,k);

    % Re-evaluate the closed loop; tau, ctrl and dist are now exposed.
    [~, kin_k, tau_k, ctrl_k, dist_k] = closed_loop_dynamics(tk, Xk, params);
    ref_k = params.ref_fcn(tk);   % selected reference

    R_0k = reshape(Xk(4:12), 3, 3);
    r_0k = Xk(1:3);
    x_ck = r_0k + R_0k * kin_k.r_0c_0;
    r_ek = r_0k + R_0k * kin_k.r_0e_0;

    % --- recorded control ---
    log.tau(:,k) = tau_k;                 % physical generalized control
    log.u(:,k)   = ctrl_k.u;              % transformed control [u1;u2;u3]

    % --- recorded positions ---
    log.r0(:,k)  = r_0k;
    log.rc(:,k)  = x_ck;
    log.re(:,k)  = r_ek;

    % --- recorded tracking errors ---
    R0c           = ctrl_k.R0c;
    log.e_x(:,k)  = x_ck - ref_k.x_cd;
    log.e_R(:,k)  = 0.5 * vee(R0c.'*R_0k - R_0k.'*R0c);
    log.e_xE(:,k) = ctrl_k.e_y(1:3);
    log.e_yaw(k)  = ctrl_k.e_y(4);

    % --- recorded disturbance: TRUE (from the plant) and GMO ESTIMATE ---
    % (both zero in 'nominal' mode up to observer residuals)
    log.d_t(:,k)   = dist_k.d_t;          % ground truth (plant)
    log.d_r(:,k)   = dist_k.d_r;
    log.d_rho(:,k) = dist_k.d_rho;
    log.d_t_hat(:,k)   = ctrl_k.d_t_hat;  % GMO estimate (controller)
    log.d_r_hat(:,k)   = ctrl_k.d_r_hat;
    log.d_rho_hat(:,k) = ctrl_k.d_rho_hat;
    log.w_e(:,k)   = dist_k.w_e;
end
fprintf('Recording complete.\n\n');

% ========================================================================
% 7.  Plots + video (saved to ./image/ and ./video/, tagged by traj + mode)
% ========================================================================
% plot_result 3rd arg picks figures (subset of 1:4 = errors|states|control|3D);
% omit for all four.  generate_video animates the 3D scene (Figure 4).
plot_result(log, params);
% generate_video(log, params);
