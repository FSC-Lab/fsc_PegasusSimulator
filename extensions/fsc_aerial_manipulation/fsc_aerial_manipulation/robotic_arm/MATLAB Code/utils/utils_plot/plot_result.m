function plot_result(log, params, figsel)
%PLOT_RESULT  Plot a closed-loop aerial-manipulator run from RECORDED signals.
%
%   plot_result(log, params) consumes the signal log produced in main_sim
%   (states + physical control tau recorded at every accepted integration
%   step) and produces four figures:
%
%     Figure 1 -- tracking-error curves:
%                 (a) drone CoM position error  e_x   = x_c - x_{c,d}
%                 (b) drone attitude error      e_R
%                 (c) arm end-effector pos error e_xE  = r_e - r_{e,d}
%                 (d) arm yaw error             e_{R_E,3}
%     Figure 2 -- recorded STATES:
%                 base position r_0, joint angles q, base linear velocity
%                 v_0, base angular velocity omega_0, joint rates qdot.
%     Figure 3 -- recorded CONTROL:
%                 (a) thrust u1, (b) base force tau(1:3),
%                 (c) base moment tau(4:6), (d) joint torques tau(7:6+n).
%     Figure 4 -- 3D trajectories of base / CoM / end-effector, with the
%                 FULL aerial manipulator drawn at the final pose and a
%                 faint skeleton at the initial pose.
%     Figure 5 -- GMO disturbance estimation (2 x 3): the TOP row overlays the
%                 true (solid) and GMO-estimated (dashed) transformed
%                 disturbance in the CoM (d_t), platform (d_r) and arm (d_rho)
%                 channels; the BOTTOM row is the estimation error d - d_hat in
%                 each channel.  (Drawn only if the log carries the GMO
%                 estimates d_t_hat / d_r_hat / d_rho_hat.)
%
%   plot_result(log, params, figsel) draws only the figures whose indices are
%   listed in figsel (a subset of 1:5, e.g. [1 5] or 3).  Omitted or empty ->
%   all five.  Only the figures actually drawn are saved, into a per-run
%   subfolder  ./image/<trajtype>_<mode>/  (created on demand).
%
%   INPUT  log : struct from main_sim with fields
%                t   (1 x N)            time stamps
%                X   ((18+2n) x N)      recorded states
%                tau ((6+n) x N)        physical control [F0(3);M0(3);tau_q(n)]
%                u   ((4+n) x N)        transformed control [u1;u2;u3]
%                r0, rc, re (3 x N)     base / CoM / EE positions (inertial)
%                e_x, e_R, e_xE (3 x N), e_yaw (1 x N)   tracking errors
%                d_t, d_r (3 x N), d_rho (n x N)         TRUE transformed dist.
%                d_t_hat, d_r_hat (3 x N), d_rho_hat (n x N)  GMO estimates
%
%   Requires on the path:  dynamics, hat   (only for drawing the body at the
%   start/end poses; no per-step controller recomputation is done here).

n = params.n;
t = log.t(:).';
N = numel(t);

% ---------------- Unpack recorded states ---------------------------------
r0_state = log.X(1:3, :);                 % base position
q_state  = log.X(13:12+n, :);             % joint angles
v0_state = log.X(13+n:15+n, :);           % base linear velocity (body)
om_state = log.X(16+n:18+n, :);           % base angular velocity (body)
qd_state = log.X(19+n:18+2*n, :);         % joint rates

% ---------------- Unpack recorded control --------------------------------
u1_h     = log.u(1, :);                   % thrust (scalar)
tau_F    = log.tau(1:3, :);               % base force
tau_M    = log.tau(4:6, :);               % base moment
tau_q    = log.tau(7:6+n, :);             % joint torques

cmap = lines(max(n,3));                    % consistent colors for joints

% ---------------- Run descriptors (mode, trajectory type) ----------------
% Used to (a) draw the payload cube only in the 'payload' case (the other
% disturbance modes apply an external force, not a hanging mass) and
% (b) tag the saved image filenames so different runs do not overwrite.
mode = 'nominal';
if isfield(params,'mode') && ~isempty(params.mode), mode = lower(params.mode); end
is_payload = strcmpi(mode, 'payload');
if isfield(params,'traj_tag') && ~isempty(params.traj_tag)
    ttype = params.traj_tag;                        % set by main_sim (both modes)
else
    try
        cfg   = plan_locked_trajectory('config', params);   % has .type (hover/line/...)
        ttype = cfg.type;
    catch
        ttype = 'traj';
    end
end
run_tag = sprintf('%s_%s', ttype, mode);             % e.g. 'line_payload'

% ---------------- Image output folder ------------------------------------
% Figures go into a per-run SUBFOLDER  image/<trajtype>_<mode>/  (e.g.
% image/hover_sine/), created on demand so different runs do not overwrite
% one another.  mkdir creates the parent image/ too if it does not exist.
img_dir = fullfile('image', run_tag);
if ~exist(img_dir, 'dir'), mkdir(img_dir); end

% ---------------- Figure selection ---------------------------------------
% Optional 3rd arg figsel: which figures to draw (subset of 1:5).  Default all.
%   1 = tracking errors, 2 = states, 3 = control, 4 = 3D trajectory,
%   5 = GMO disturbance estimation.
if nargin < 3 || isempty(figsel), figsel = 1:5; end
want = @(k) any(figsel(:) == k);          % true if figure k is requested
f1 = []; f2 = []; f3 = []; f4 = []; f5 = [];   % handles (empty = not drawn)

% ========================================================================
% FIGURE 1 -- Tracking errors
% ========================================================================
if want(1)
f1 = figure('Color','w', 'Name','Tracking Errors', 'Position', [40 80 1000 720]);

subplot(2,2,1);
plot(t, log.e_x(1,:), 'r-', t, log.e_x(2,:), 'g-', t, log.e_x(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{e}_x$ (m)','Interpreter','latex');
title('Drone CoM position error','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(2,2,2);
plot(t, log.e_R(1,:), 'r-', t, log.e_R(2,:), 'g-', t, log.e_R(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{e}_R$','Interpreter','latex');
title('Drone attitude error','Interpreter','latex');
legend({'$e_{R,1}$','$e_{R,2}$','$e_{R,3}$'}, 'Interpreter','latex', 'Location','best');

subplot(2,2,3);
plot(t, log.e_xE(1,:), 'r-', t, log.e_xE(2,:), 'g-', t, log.e_xE(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{e}_{x_E}$ (m)','Interpreter','latex');
title('Arm end-effector position error','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(2,2,4);
plot(t, log.e_yaw, 'k-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$e_{R_E,3}$','Interpreter','latex');
title('Arm yaw error','Interpreter','latex');
end   % want(1)

% ========================================================================
% FIGURE 2 -- Recorded states
% ========================================================================
if want(2)
f2 = figure('Color','w', 'Name','Recorded States', 'Position', [70 60 1100 760]);

subplot(3,2,1);
plot(t, r0_state(1,:), 'r-', t, r0_state(2,:), 'g-', t, r0_state(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$r_0$ (m)','Interpreter','latex');
title('Base position $r_0$','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(3,2,2);
hold on;
for j = 1:n
    plot(t, q_state(j,:), '-', 'Color', cmap(j,:), 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$q$ (rad)','Interpreter','latex');
title('Joint angles $q$','Interpreter','latex');
legend(arrayfun(@(j) sprintf('$q_{%d}$',j), 1:n, 'UniformOutput', false), ...
       'Interpreter','latex', 'Location','best');

subplot(3,2,3);
plot(t, v0_state(1,:), 'r-', t, v0_state(2,:), 'g-', t, v0_state(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$v_0$ (m/s)','Interpreter','latex');
title('Base linear velocity $v_0$ (body)','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(3,2,4);
plot(t, om_state(1,:), 'r-', t, om_state(2,:), 'g-', t, om_state(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\omega_0$ (rad/s)','Interpreter','latex');
title('Base angular velocity $\omega_0$ (body)','Interpreter','latex');
legend({'$\omega_x$','$\omega_y$','$\omega_z$'}, 'Interpreter','latex', 'Location','best');

subplot(3,2,5);
hold on;
for j = 1:n
    plot(t, qd_state(j,:), '-', 'Color', cmap(j,:), 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\dot{q}$ (rad/s)','Interpreter','latex');
title('Joint rates $\dot{q}$','Interpreter','latex');
legend(arrayfun(@(j) sprintf('$\\dot{q}_{%d}$',j), 1:n, 'UniformOutput', false), ...
       'Interpreter','latex', 'Location','best');
end   % want(2)

% ========================================================================
% FIGURE 3 -- Recorded control (transformed thrust + physical tau)
% ========================================================================
if want(3)
f3 = figure('Color','w', 'Name','Recorded Control', 'Position', [100 50 1000 760]);

subplot(2,2,1);
plot(t, u1_h, 'k-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$u_1$ (N)','Interpreter','latex');
title('Thrust $u_1$','Interpreter','latex');

subplot(2,2,2);
plot(t, tau_F(1,:), 'r-', t, tau_F(2,:), 'g-', t, tau_F(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\tau_{F}~(N)$','Interpreter','latex');
title('Base force $\tau_{1:3}=(T^\top u)_{1:3}$','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(2,2,3);
plot(t, tau_M(1,:), 'r-', t, tau_M(2,:), 'g-', t, tau_M(3,:), 'b-', 'LineWidth', 1.5);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\tau_{M}~(N \cdot m)$','Interpreter','latex');
title('Base moment $\tau_{4:6}$','Interpreter','latex');
legend({'$x$','$y$','$z$'}, 'Interpreter','latex', 'Location','best');

subplot(2,2,4);
hold on;
for j = 1:n
    plot(t, tau_q(j,:), '-', 'Color', cmap(j,:), 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('${\tau}_q~(N \cdot m)$','Interpreter','latex');
title('Joint torques $\tau_{7:6+n}$','Interpreter','latex');
legend(arrayfun(@(j) sprintf('$\tau_{q,%d}$',j), 1:n, 'UniformOutput', false), ...
       'Interpreter','latex', 'Location','best');
end   % want(3)

% ========================================================================
% FIGURE 4 -- 3D trajectories + full aerial manipulator
% ========================================================================
if want(4)
f4 = figure('Color','w', 'Name','3D Trajectory', 'Position', [120 40 980 800]);
hold on; grid on; box on; axis equal;
xlabel('$x_I$ (m)','Interpreter','latex');
ylabel('$y_I$ (m)','Interpreter','latex');
zlabel('$z_I$ (m)','Interpreter','latex');
title('Closed-loop 3D trajectory','Interpreter','latex','FontWeight','bold');
view(35, 18); set(gca, 'FontSize', 10);

% --- trajectory curves (from recorded positions) ---
h_base = plot3(log.r0(1,:), log.r0(2,:), log.r0(3,:), '-',  'Color',[0.20 0.20 0.20], 'LineWidth',1.6);
h_com  = plot3(log.rc(1,:), log.rc(2,:), log.rc(3,:), '--', 'Color',[0.10 0.65 0.20], 'LineWidth',1.6);
h_ee   = plot3(log.re(1,:), log.re(2,:), log.re(3,:), '-',  'Color',[0.85 0.45 0.05], 'LineWidth',1.6);

% --- start / end base markers ---
plot3(log.r0(1,1),   log.r0(2,1),   log.r0(3,1),   'o', 'MarkerSize',8, 'MarkerFaceColor',[0.2 0.6 1.0], 'MarkerEdgeColor','k');
plot3(log.r0(1,end), log.r0(2,end), log.r0(3,end), 's', 'MarkerSize',9, 'MarkerFaceColor',[1.0 0.3 0.3], 'MarkerEdgeColor','k');

% --- aerial manipulator: faint skeleton at initial pose, full body at final ---
[~, kin0]   = dynamics(log.X(:,1),   params);
[~, kinEnd] = dynamics(log.X(:,end), params);
draw_skeleton(kin0, params, 0.35);                 % faint skeleton (initial)
draw_full_manipulator(kinEnd, params);             % full render (final)

% --- payload ('payload' mode only): small cube hanging at the EE ----------
leg_h   = [h_base, h_com, h_ee];
leg_txt = {'Base $r_0$','System CoM $x_c$','End-effector $r_e$'};
if is_payload
    R0_end = reshape(log.X(4:12,end), 3, 3);
    [Uo,~,Vo] = svd(R0_end);  R0_end = Uo*Vo';     % re-orthonormalize
    R_e_end   = R0_end * kinEnd.R_e_0;             % EE orientation in {I}
    s_cube    = payload_cube_size(params);
    c_cube    = log.re(:,end) + [0;0;-s_cube/2];   % hang just below the EE
    h_pay     = draw_payload_cube(c_cube, R_e_end, s_cube);
    leg_h(end+1)   = h_pay;
    leg_txt{end+1} = sprintf('Payload (%.0f g)', 1e3*params.payload_mass);
end

% --- legend ---
legend(leg_h, leg_txt, ...
       'Interpreter','latex', 'Location','eastoutside', 'FontSize',9);
hold off;
end   % want(4)

% ========================================================================
% FIGURE 5 -- GMO disturbance estimation (true vs estimate, and error)
% ========================================================================
% Top row  : true (solid) vs GMO estimate (dashed) of the transformed
%            disturbance in each channel -- CoM/translation d_t (N),
%            platform/rotation d_r (N*m), arm d_rho (N*m).
% Bottom row: the estimation error  d_tilde = d - d_hat  in each channel.
has_dhat = isfield(log,'d_t_hat') && isfield(log,'d_r_hat') && isfield(log,'d_rho_hat');
if want(5) && ~has_dhat
    warning('plot_result:noGMO', ...
            'Figure 5 skipped: log has no GMO estimates (d_t_hat/d_r_hat/d_rho_hat).');
end
if want(5) && has_dhat
f5 = figure('Color','w', 'Name','GMO Disturbance Estimation', 'Position', [140 40 1280 720]);
rgb = {'r','g','b'};                              % x/y/z component colors

% ---------- (1,1) CoM channel d_t : true vs estimate ----------
subplot(2,3,1); hold on;
for i = 1:3
    plot(t, log.d_t(i,:),     '-',  'Color', rgb{i}, 'LineWidth', 1.5);
    plot(t, log.d_t_hat(i,:), '--', 'Color', rgb{i}, 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{d}_t$ (N)','Interpreter','latex');
title('CoM channel $\mathbf{d}_t$ (solid: true, dashed: est)','Interpreter','latex');
legend({'$d_{t,x}$','$\hat{d}_{t,x}$','$d_{t,y}$','$\hat{d}_{t,y}$','$d_{t,z}$','$\hat{d}_{t,z}$'}, ...
       'Interpreter','latex','Location','best','FontSize',8,'NumColumns',3);

% ---------- (1,2) platform channel d_r : true vs estimate ----------
subplot(2,3,2); hold on;
for i = 1:3
    plot(t, log.d_r(i,:),     '-',  'Color', rgb{i}, 'LineWidth', 1.5);
    plot(t, log.d_r_hat(i,:), '--', 'Color', rgb{i}, 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{d}_r~(N \cdot m)$','Interpreter','latex');
title('Platform channel $\mathbf{d}_r$ (solid: true, dashed: est)','Interpreter','latex');
legend({'$d_{r,x}$','$\hat{d}_{r,x}$','$d_{r,y}$','$\hat{d}_{r,y}$','$d_{r,z}$','$\hat{d}_{r,z}$'}, ...
       'Interpreter','latex','Location','best','FontSize',8,'NumColumns',3);

% ---------- (1,3) arm channel d_rho : true vs estimate ----------
subplot(2,3,3); hold on;
leg_rho = cell(1,2*n);
for j = 1:n
    plot(t, log.d_rho(j,:),     '-',  'Color', cmap(j,:), 'LineWidth', 1.5);
    plot(t, log.d_rho_hat(j,:), '--', 'Color', cmap(j,:), 'LineWidth', 1.5);
    leg_rho{2*j-1} = sprintf('$d_{\\rho,%d}$', j);
    leg_rho{2*j}   = sprintf('$\\hat{d}_{\\rho,%d}$', j);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\mathbf{d}_\rho~(N \cdot m)$','Interpreter','latex');
title('Arm channel $\mathbf{d}_\rho$ (solid: true, dashed: est)','Interpreter','latex');
legend(leg_rho, 'Interpreter','latex','Location','best','FontSize',8,'NumColumns',2);

% ---------- (2,1) CoM channel estimation error ----------
subplot(2,3,4); hold on;
for i = 1:3
    plot(t, log.d_t(i,:) - log.d_t_hat(i,:), '-', 'Color', rgb{i}, 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\tilde{\mathbf{d}}_t$ (N)','Interpreter','latex');
title('CoM channel estimation error','Interpreter','latex');
legend({'$\tilde{d}_{t,x}$','$\tilde{d}_{t,y}$','$\tilde{d}_{t,z}$'}, ...
       'Interpreter','latex','Location','best','FontSize',8);

% ---------- (2,2) platform channel estimation error ----------
subplot(2,3,5); hold on;
for i = 1:3
    plot(t, log.d_r(i,:) - log.d_r_hat(i,:), '-', 'Color', rgb{i}, 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\tilde{\mathbf{d}}_r~(N \cdot m)$','Interpreter','latex');
title('Platform channel estimation error','Interpreter','latex');
legend({'$\tilde{d}_{r,x}$','$\tilde{d}_{r,y}$','$\tilde{d}_{r,z}$'}, ...
       'Interpreter','latex','Location','best','FontSize',8);

% ---------- (2,3) arm channel estimation error ----------
subplot(2,3,6); hold on;
for j = 1:n
    plot(t, log.d_rho(j,:) - log.d_rho_hat(j,:), '-', 'Color', cmap(j,:), 'LineWidth', 1.5);
end
hold off; grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('$\tilde{\mathbf{d}}_\rho~(N \cdot m)$','Interpreter','latex');
title('Arm channel estimation error','Interpreter','latex');
legend(arrayfun(@(j) sprintf('$\\tilde{d}_{\\rho,%d}$',j), 1:n, 'UniformOutput', false), ...
       'Interpreter','latex','Location','best','FontSize',8,'NumColumns',2);

% ---- overall banner (mode + GMO on/off) ----
gmo_on  = ~isfield(params,'use_gmo') || isempty(params.use_gmo) || params.use_gmo;
gmo_lbl = {'OFF','ON'};
sgtitle(sprintf('GMO disturbance estimation  |  mode: %s  |  GMO: %s', ...
        mode, gmo_lbl{gmo_on+1}), 'Interpreter','none', 'FontWeight','bold');
end   % want(5)

% ========================================================================
% SAVE FIGURES  ->  image/<trajtype>_<mode>/<name>.png
% ========================================================================
% Only the figures that were actually drawn (see figsel) are saved.  The run
% identity lives in the subfolder name (img_dir), so the filenames are plain.
nsaved = 0;
if ~isempty(f1), save_fig(f1, fullfile(img_dir, 'tracking_errors.png')); nsaved = nsaved+1; end
if ~isempty(f2), save_fig(f2, fullfile(img_dir, 'states.png'));          nsaved = nsaved+1; end
if ~isempty(f3), save_fig(f3, fullfile(img_dir, 'control.png'));         nsaved = nsaved+1; end
if ~isempty(f4), save_fig(f4, fullfile(img_dir, 'trajectory3d.png'));    nsaved = nsaved+1; end
if ~isempty(f5), save_fig(f5, fullfile(img_dir, 'gmo_dist_est.png'));    nsaved = nsaved+1; end
fprintf('Saved %d figure(s) to ./%s/\n', nsaved, strrep(img_dir, '\', '/'));
end

% ========================================================================
%  payload_cube_size  --  edge length (m) of the drawn payload cube
%       Scales gently with mass (cube-root) so heavier loads look bigger,
%       anchored at 0.05 m for a 0.2 kg payload.  Purely cosmetic.
% ========================================================================
function s = payload_cube_size(params)
m_p = 0.2;
if isfield(params,'payload_mass') && ~isempty(params.payload_mass)
    m_p = params.payload_mass;
end
s = 0.05 * (max(m_p,1e-3)/0.2)^(1/3);
end

% ========================================================================
%  draw_payload_cube  --  small solid cube centered at c, oriented by R.
%       Returns a patch handle (for the legend).
% ========================================================================
function h = draw_payload_cube(c, R, s, faceColor)
if nargin < 4, faceColor = [0.55 0.35 0.15]; end   % crate brown
% unit cube vertices centered at origin (8 x 3), scaled, rotated, translated
V = 0.5*[ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
          -1 -1  1;  1 -1  1;  1  1  1; -1  1  1 ];
V = (R * (s * V.')).' + c(:).';                    % 8 x 3 in {I}
F = [ 1 2 3 4;  5 6 7 8;  1 2 6 5;  2 3 7 6;  3 4 8 7;  4 1 5 8 ];
h = patch('Vertices', V, 'Faces', F, ...
          'FaceColor', faceColor, 'FaceAlpha', 0.85, ...
          'EdgeColor', 'k', 'LineWidth', 1.0);
end

% ========================================================================
%  save_fig  --  robust figure export (exportgraphics if available)
% ========================================================================
function save_fig(figh, fname)
try
    exportgraphics(figh, fname, 'Resolution', 150);
catch
    try
        print(figh, fname, '-dpng', '-r150');
    catch err
        warning('plot_result:save', 'Could not save %s (%s).', fname, err.message);
    end
end
end

% ========================================================================
%  draw_full_manipulator  --  full aerial-manipulator body in current axes
%       (quadrotor arms, propeller disks, body frames, links, CoM, EE)
%       Adapted from visualize_aerial_manipulator (no figure/legend setup).
% ========================================================================
function draw_full_manipulator(kin, params, varargin)
p = inputParser;
addParameter(p, 'AxisLength',    0.06);
addParameter(p, 'ArmLength',     0.650/2);
addParameter(p, 'PropRadius',    15*0.0254/2);
addParameter(p, 'AxisLineWidth', 2.0);
addParameter(p, 'DroneLineWidth',2.0);
addParameter(p, 'LinkLineWidth', 2.0);
addParameter(p, 'ShowFrames',    true);
parse(p, varargin{:});
axis_len = p.Results.AxisLength;
arm_len  = p.Results.ArmLength;
prop_rad = p.Results.PropRadius;
axis_lw  = p.Results.AxisLineWidth;
drone_lw = p.Results.DroneLineWidth;
link_lw  = p.Results.LinkLineWidth;
showFr   = p.Results.ShowFrames;

n   = params.n;
r_0 = kin.r_0;
R_0 = kin.R_0;
[U,~,Vs] = svd(R_0);  R_0 = U*Vs';     % re-orthonormalize (integration drift)

O_I = zeros(3, n+1);  R_I = zeros(3,3,n+1);
for k = 1:n+1
    O_I(:,k)   = r_0 + R_0 * kin.r_link_ends(:,k);
    R_I(:,:,k) = R_0 * kin.R_i_0(:,:,k);
end
r_e_I = r_0 + R_0 * kin.r_0e_0;
r_c_I = r_0 + R_0 * kin.r_0c_0;

joint_color = [0.95 0.55 0.10];
link_color  = [0.30 0.30 0.30];
prop_color  = [0.30 0.55 0.95];
prop_face   = [0.65 0.80 0.98];

% ---- quadrotor motors (X-config) ----
ang = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
motor_0 = zeros(3,4);  motor_I = zeros(3,4);
for m = 1:4
    motor_0(:,m) = arm_len * [cos(ang(m)); sin(ang(m)); 0];
    motor_I(:,m) = r_0 + R_0 * motor_0(:,m);
end

% ---- crossing arms ----
plot3([motor_I(1,1) motor_I(1,3)], [motor_I(2,1) motor_I(2,3)], [motor_I(3,1) motor_I(3,3)], 'k-', 'LineWidth', drone_lw);
plot3([motor_I(1,2) motor_I(1,4)], [motor_I(2,2) motor_I(2,4)], [motor_I(3,2) motor_I(3,4)], 'k-', 'LineWidth', drone_lw);

% ---- motor dots ----
plot3(motor_I(1,:), motor_I(2,:), motor_I(3,:), 'ko', 'MarkerSize', 9, 'MarkerFaceColor', [0.15 0.15 0.15]);

% ---- propeller disks ----
theta_c = linspace(0, 2*pi, 60);
for m = 1:4
    circ_local = [prop_rad*cos(theta_c); prop_rad*sin(theta_c); zeros(1,numel(theta_c))];
    circ_I     = r_0 + R_0 * (motor_0(:,m) + circ_local);
    patch(circ_I(1,:), circ_I(2,:), circ_I(3,:), prop_face, ...
          'FaceAlpha',0.25, 'EdgeColor',prop_color, 'EdgeAlpha',0.85, 'LineWidth',1.4);
end

% ---- manipulator links (bent link 2 drawn as two segments) ----
Plink = arm_link_polyline(O_I, R_I, params.l_i);
plot3(Plink(1,:), Plink(2,:), Plink(3,:), 'Color', link_color, 'LineWidth', link_lw);

% ---- joint dots ----
for k = 1:n+1
    plot3(O_I(1,k), O_I(2,k), O_I(3,k), 'o', 'MarkerSize', 9, ...
          'MarkerFaceColor', joint_color, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
end

% ---- end-effector + system CoM ----
plot3(r_e_I(1), r_e_I(2), r_e_I(3), 'p', 'MarkerSize', 14, ...
      'MarkerFaceColor', [1.00 0.85 0.10], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
plot3(r_c_I(1), r_c_I(2), r_c_I(3), '^', 'MarkerSize', 10, ...
      'MarkerFaceColor', [0.10 0.65 0.20], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

% ---- body frames (RGB triads at {0}..{n}) ----
if showFr
    draw_frame(r_0, R_0, axis_len, axis_lw);
    for k = 2:n+1
        draw_frame(O_I(:,k), R_I(:,:,k), axis_len, axis_lw);
    end
end
end

% ========================================================================
%  draw_skeleton  --  lightweight manipulator skeleton (no quad body)
% ========================================================================
function draw_skeleton(kin, params, alpha)
n   = params.n;
r_0 = kin.r_0;
R_0 = kin.R_0;
[U,~,Vs] = svd(R_0);  R_0 = U*Vs';

O_I = zeros(3, n+1);  R_I = zeros(3,3,n+1);
for k = 1:n+1
    O_I(:,k)   = r_0 + R_0 * kin.r_link_ends(:,k);
    R_I(:,:,k) = R_0 * kin.R_i_0(:,:,k);
end
r_e_I = r_0 + R_0 * kin.r_0e_0;

link_color  = [0.30 0.30 0.30];
joint_color = [0.95 0.55 0.10];

% links (bent link 2 drawn as two segments)
Plink = arm_link_polyline(O_I, R_I, params.l_i);
plot3(Plink(1,:), Plink(2,:), Plink(3,:), '-', 'Color', [link_color, alpha], 'LineWidth', 2.0);
for k = 1:n+1
    plot3(O_I(1,k), O_I(2,k), O_I(3,k), 'o', 'MarkerSize', 5, ...
          'MarkerFaceColor', joint_color, 'MarkerEdgeColor', 'k');
end
plot3(r_0(1), r_0(2), r_0(3), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', [0.15 0.15 0.15]);
plot3(r_e_I(1), r_e_I(2), r_e_I(3), 'p', 'MarkerSize', 10, ...
      'MarkerFaceColor', [1.00 0.85 0.10], 'MarkerEdgeColor', 'k');
end

% ========================================================================
%  draw_frame  --  RGB axis triad (x=red, y=green, z=blue)
% ========================================================================
function draw_frame(origin, R, L, lw)
ax_color = {'r','g','b'};
for ax = 1:3
    quiver3(origin(1), origin(2), origin(3), ...
            L*R(1,ax), L*R(2,ax), L*R(3,ax), 0, ...
            'Color', ax_color{ax}, 'LineWidth', lw, 'MaxHeadSize', 0.6);
end
end

% ========================================================================
%  vee  --  inverse of hat: so(3) (3x3 skew) -> R^3
% ========================================================================
function v = vee(S)
v = [S(3,2); S(1,3); S(2,1)];
end