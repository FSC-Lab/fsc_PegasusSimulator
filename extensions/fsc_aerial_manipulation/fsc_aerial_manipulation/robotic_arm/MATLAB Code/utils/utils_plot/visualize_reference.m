function visualize_reference(params, varargin)
%VISUALIZE_REFERENCE  Debug-plot the desired (reference) trajectory.
%
%   visualize_reference(params) samples plan_locked_trajectory over the maneuver
%   horizon [0, T] (T from plan_locked_trajectory's config) and produces four figures so the reference can
%   be inspected BEFORE running the closed-loop simulation:
%
%     Figure 1 -- System-CoM reference x_cd and its time derivatives:
%                 position, velocity, acceleration, jerk (d3), snap (d4),
%                 plus a panel of derivative magnitudes.
%     Figure 2 -- End-effector reference r_ed and its time derivatives:
%                 position, velocity, acceleration, plus magnitudes.
%     Figure 3 -- Heading directions (unit vectors) and their derivatives:
%                 base heading b1_d (left column), EE heading b1_de (right
%                 column); rows = value, 1st derivative, 2nd derivative.
%     Figure 4 -- 3D view: x_cd and r_ed position CURVES, with sampled
%                 heading VECTORS b1_d (base) and b1_de (EE) drawn as quivers
%                 at sampled time instants.
%
%   It ALSO prints a finite-difference consistency check: every analytic
%   derivative is compared against the central difference of the signal one
%   order below it.  All numbers should be ~0.  (Small spikes near t=0 and
%   t=T are expected and harmless: the min-jerk time-scaling is only C^2 at
%   the rest endpoints, so the analytic jerk/snap jump there while a finite
%   difference smears the jump -- the check trims a small margin at each end.)
%
%   visualize_reference(params, 'Name', value, ...) options:
%       'N'          number of dense samples for the curves    (default 600)
%       'NQuiver'    number of sampled instants for arrows      (default 14)
%       'QuiverFrac' arrow length as a fraction of bbox diag    (default 0.10)
%
%   Requires on the path:  plan_locked_trajectory.

% ---------------- options ------------------------------------------------
p = inputParser;
addParameter(p, 'N',          600);
addParameter(p, 'NQuiver',    14);
addParameter(p, 'QuiverFrac', 0.10);
parse(p, varargin{:});
N  = p.Results.N;
NQ = p.Results.NQuiver;
qf = p.Results.QuiverFrac;

cfg = plan_locked_trajectory('config', params);    % active trajectory config
T   = cfg.T;                                    % duration
t = linspace(0, T, N);

% ---------------- sample the reference -----------------------------------
x_cd    = zeros(3,N);  x_cd_d1 = zeros(3,N);  x_cd_d2 = zeros(3,N);
x_cd_d3 = zeros(3,N);  x_cd_d4 = zeros(3,N);
r_ed    = zeros(3,N);  r_ed_d1 = zeros(3,N);  r_ed_d2 = zeros(3,N);
b1d     = zeros(3,N);  b1d_d1  = zeros(3,N);  b1d_d2  = zeros(3,N);
b1de    = zeros(3,N);  b1de_d1 = zeros(3,N);  b1de_d2 = zeros(3,N);

for k = 1:N
    ref = plan_locked_trajectory(t(k), params);
    x_cd(:,k)    = ref.x_cd;      x_cd_d1(:,k) = ref.x_cd_dot;
    x_cd_d2(:,k) = ref.x_cd_ddot; x_cd_d3(:,k) = ref.x_cd_d3;
    x_cd_d4(:,k) = ref.x_cd_d4;
    r_ed(:,k)    = ref.r_ed;      r_ed_d1(:,k) = ref.r_ed_dot;
    r_ed_d2(:,k) = ref.r_ed_ddot;
    b1d(:,k)     = ref.b1_d;      b1d_d1(:,k)  = ref.b1_d_dot;
    b1d_d2(:,k)  = ref.b1_d_ddot;
    b1de(:,k)    = ref.b1_de;     b1de_d1(:,k) = ref.b1_de_dot;
    b1de_d2(:,k) = ref.b1_de_ddot;
end

% ---------------- finite-difference consistency check --------------------
fprintf('=== Reference-trajectory FD consistency check (type = ''%s'', T = %.2f s) ===\n', ...
        cfg.type, T);
fprintf('  (central difference of order k-1  vs  analytic order k; should be ~0)\n');
fd_check('d/dt x_cd      vs x_cd_dot ', x_cd,    x_cd_d1, t);
fd_check('d/dt x_cd_dot  vs x_cd_ddot', x_cd_d1, x_cd_d2, t);
fd_check('d/dt x_cd_ddot vs x_cd_d3  ', x_cd_d2, x_cd_d3, t);
fd_check('d/dt x_cd_d3   vs x_cd_d4  ', x_cd_d3, x_cd_d4, t);
fd_check('d/dt r_ed      vs r_ed_dot ', r_ed,    r_ed_d1, t);
fd_check('d/dt r_ed_dot  vs r_ed_ddot', r_ed_d1, r_ed_d2, t);
fd_check('d/dt b1_d      vs b1_d_dot ', b1d,     b1d_d1,  t);
fd_check('d/dt b1_d_dot  vs b1_d_ddot', b1d_d1,  b1d_d2,  t);
fd_check('d/dt b1_de     vs b1_de_dot', b1de,    b1de_d1, t);
fd_check('d/dt b1_de_dot vs b1_de_ddt', b1de_d1, b1de_d2, t);
fprintf('  max | ||b1_d|| - 1 |   = %.3e   (heading should stay unit-norm)\n', ...
        max(abs(vecnorm(b1d)  - 1)));
fprintf('  max | ||b1_de|| - 1 |  = %.3e\n\n', max(abs(vecnorm(b1de) - 1)));

% ========================================================================
% FIGURE 1 -- System-CoM reference x_cd and derivatives
% ========================================================================
figure('Color','w','Name','Reference: CoM x_{cd}','Position',[40 80 1150 720]);
subplot(2,3,1); comp3(t, x_cd,    'CoM position $x_{cd}$',          '$x_{cd}$ (m)');
subplot(2,3,2); comp3(t, x_cd_d1, 'CoM velocity $\dot{x}_{cd}$',    '$\dot{x}_{cd}$ (m/s)');
subplot(2,3,3); comp3(t, x_cd_d2, 'CoM accel.\ $\ddot{x}_{cd}$',    '$\ddot{x}_{cd}$ (m/s$^2$)');
subplot(2,3,4); comp3(t, x_cd_d3, 'CoM jerk $x_{cd}^{(3)}$',        '$x_{cd}^{(3)}$ (m/s$^3$)');
subplot(2,3,5); comp3(t, x_cd_d4, 'CoM snap $x_{cd}^{(4)}$',        '$x_{cd}^{(4)}$ (m/s$^4$)');
subplot(2,3,6);
plot(t, vecnorm(x_cd_d1),'-', t, vecnorm(x_cd_d2),'-', ...
     t, vecnorm(x_cd_d3),'-', t, vecnorm(x_cd_d4),'-', 'LineWidth',1.4);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('magnitude','Interpreter','latex');
title('CoM derivative magnitudes','Interpreter','latex');
legend({'$\|\dot{x}\|$','$\|\ddot{x}\|$','$\|x^{(3)}\|$','$\|x^{(4)}\|$'}, ...
       'Interpreter','latex','Location','best');

% ========================================================================
% FIGURE 2 -- End-effector reference r_ed and derivatives
% ========================================================================
figure('Color','w','Name','Reference: EE r_{ed}','Position',[70 60 1000 700]);
subplot(2,2,1); comp3(t, r_ed,    'EE position $r_{ed}$',        '$r_{ed}$ (m)');
subplot(2,2,2); comp3(t, r_ed_d1, 'EE velocity $\dot{r}_{ed}$',  '$\dot{r}_{ed}$ (m/s)');
subplot(2,2,3); comp3(t, r_ed_d2, 'EE accel.\ $\ddot{r}_{ed}$',  '$\ddot{r}_{ed}$ (m/s$^2$)');
subplot(2,2,4);
plot(t, vecnorm(r_ed_d1),'-', t, vecnorm(r_ed_d2),'-', 'LineWidth',1.4);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel('magnitude','Interpreter','latex');
title('EE derivative magnitudes','Interpreter','latex');
legend({'$\|\dot{r}_{ed}\|$','$\|\ddot{r}_{ed}\|$'}, ...
       'Interpreter','latex','Location','best');

% ========================================================================
% FIGURE 3 -- Heading directions b1_d / b1_de and derivatives (time series)
% ========================================================================
figure('Color','w','Name','Reference: headings b1_d / b1_de','Position',[100 50 1050 760]);
subplot(3,2,1); comp3(t, b1d,     'Base heading $b_{1d}$',                 '$b_{1d}$');
subplot(3,2,2); comp3(t, b1de,    'EE heading $b_{1d}^{e}$',               '$b_{1d}^{e}$');
subplot(3,2,3); comp3(t, b1d_d1,  'Base heading rate $\dot{b}_{1d}$',      '$\dot{b}_{1d}$ (1/s)');
subplot(3,2,4); comp3(t, b1de_d1, 'EE heading rate $\dot{b}_{1d}^{e}$',    '$\dot{b}_{1d}^{e}$ (1/s)');
subplot(3,2,5); comp3(t, b1d_d2,  'Base heading accel.\ $\ddot{b}_{1d}$',  '$\ddot{b}_{1d}$ (1/s$^2$)');
subplot(3,2,6); comp3(t, b1de_d2, 'EE heading accel.\ $\ddot{b}_{1d}^{e}$','$\ddot{b}_{1d}^{e}$ (1/s$^2$)');

% ========================================================================
% FIGURE 4 -- 3D position curves + sampled heading vectors
% ========================================================================
figure('Color','w','Name','Reference: 3D trajectory + headings','Position',[120 40 980 800]);
hold on; grid on; box on; axis equal;
xlabel('$x_I$ (m)','Interpreter','latex');
ylabel('$y_I$ (m)','Interpreter','latex');
zlabel('$z_I$ (m)','Interpreter','latex');
title('Reference trajectory: position curves + heading vectors', ...
      'Interpreter','latex','FontWeight','bold');
view(35, 18); set(gca,'FontSize',10);

% --- position curves ---
h_com = plot3(x_cd(1,:), x_cd(2,:), x_cd(3,:), '--', 'Color',[0.10 0.65 0.20], 'LineWidth',1.8);
h_ee  = plot3(r_ed(1,:), r_ed(2,:), r_ed(3,:), '-',  'Color',[0.85 0.45 0.05], 'LineWidth',1.8);

% --- start / end markers (on the CoM curve) ---
plot3(x_cd(1,1),   x_cd(2,1),   x_cd(3,1),   'o','MarkerSize',9, 'MarkerFaceColor',[0.2 0.6 1.0],'MarkerEdgeColor','k');
plot3(x_cd(1,end), x_cd(2,end), x_cd(3,end), 's','MarkerSize',10,'MarkerFaceColor',[1.0 0.3 0.3],'MarkerEdgeColor','k');

% --- arrow length from the combined bounding box ---
allP = [x_cd, r_ed];
span = max(allP,[],2) - min(allP,[],2);
L    = qf * max(norm(span), eps);          % robust if a path is degenerate (e.g. point)

% --- sample instants and draw heading quivers ---
idx = unique(round(linspace(1, N, NQ)));
h_b1d  = quiver3(x_cd(1,idx), x_cd(2,idx), x_cd(3,idx), ...
                 L*b1d(1,idx), L*b1d(2,idx), L*b1d(3,idx), 0, ...
                 'Color',[0.10 0.35 0.90], 'LineWidth',1.6, 'MaxHeadSize',0.5);
h_b1de = quiver3(r_ed(1,idx), r_ed(2,idx), r_ed(3,idx), ...
                 L*b1de(1,idx), L*b1de(2,idx), L*b1de(3,idx), 0, ...
                 'Color',[0.85 0.10 0.55], 'LineWidth',1.6, 'MaxHeadSize',0.5);

legend([h_com, h_ee, h_b1d, h_b1de], ...
       {'CoM $x_{cd}$','EE $r_{ed}$','base heading $b_{1d}$','EE heading $b_{1d}^{e}$'}, ...
       'Interpreter','latex','Location','eastoutside','FontSize',9);
hold off;
end

% ========================================================================
%  comp3  --  plot the 3 components (x/y/z) of a 3xN signal vs time
% ========================================================================
function comp3(t, M, ttl, yl)
plot(t, M(1,:), 'r-', t, M(2,:), 'g-', t, M(3,:), 'b-', 'LineWidth', 1.4);
grid on; xlabel('$t$ (s)','Interpreter','latex');
ylabel(yl,'Interpreter','latex');
title(ttl,'Interpreter','latex');
legend({'$x$','$y$','$z$'},'Interpreter','latex','Location','best');
end

% ========================================================================
%  fd_check  --  central-difference of f compared with analytic fdot.
%       Trims a 2% margin at each end (the min-jerk profile is only C^2 at
%       the rest endpoints, so analytic jerk/snap jump there).
% ========================================================================
function fd_check(name, f, fdot, t)
dt = t(2) - t(1);
df = zeros(size(f));
df(:,2:end-1) = (f(:,3:end) - f(:,1:end-2)) / (2*dt);
m   = max(1, round(0.02*size(f,2)));
idx = (1+m):(size(f,2)-m);
err = max(vecnorm(df(:,idx) - fdot(:,idx)));
fprintf('  %s   max|FD - analytic| = %.3e\n', name, err);
end