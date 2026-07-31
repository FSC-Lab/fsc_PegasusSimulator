function plan = plan_compatible_trajectory(params, opts)
%PLAN_COMPATIBLE_TRAJECTORY  Offline planner for the dynamically-compatible
%   system-CoM trajectory of an aerial manipulator + drop-in reference for
%   the closed-loop simulation.
%
%   plan = plan_compatible_trajectory(params, opts)
%
%   Runs ONCE, offline.  It PRESCRIBES the end-effector task (p_e, R_e) and the
%   platform heading, and SOLVES for the compatible CoM p_c(t) using
%       (C1) t  = (p_c_ddot + g e3)/||.||                       (thrust dir)
%       (C2) p_c = p_e + R_0 ( r_0c^0(theta) - r_0e^0(theta) )  (CoM offset)
%   R_0 is built from (t, b1_d) as in controller.m (eqs 98-101); theta is
%   recovered from R_e^0 = R_0' R_e by z-y-z decomposition with the redundancy
%   closed by (R1) b1_d prescribed and (R2) theta_2 = theta_3.  p_c is a
%   polynomial (=> C^inf), solved by a damped Picard iteration.
%
%   OUTPUT  plan.  Fields used for integration into main_sim:
%       plan.ref(tq) : function handle -> FULL controller reference struct
%                      (x_cd..x_cd_d4, b1_d(+d,dd), r_ed(+d,dd), b1_de(+d,dd)).
%                      Drop-in replacement for plan_locked_trajectory(t,params).
%       plan.X0      : consistent initial state (zero initial tracking error).
%       plan.T       : simulation horizon (s).
%       plan.q0, plan.R0_0, plan.r0_0 : the t=0 configuration.
%   Plus analysis fields: .t, .p_c, .p_e, .theta, .beta, .Pc, .diag.
%
%   No toolboxes / no external helpers required (self-contained).

% ========================================================================
% 0.  Parameters and options
% ========================================================================
if nargin < 1 || isempty(params), params = default_params(); end
if nargin < 2 || isempty(opts),   opts   = struct();         end
opts = setdef(opts, 'T',     15.0);
opts = setdef(opts, 'N',     201);
opts = setdef(opts, 'deg',   16);
opts = setdef(opts, 'maxit', 60);
opts = setdef(opts, 'tol',   1e-10);
opts = setdef(opts, 'relax', 1.0);
opts = setdef(opts, 'Nfine', 801);
opts = setdef(opts, 'plot',  true);

g  = params.g;  e3 = [0;0;1];  T = opts.T;
t  = linspace(0, T, opts.N).';

% ========================================================================
% 1.  PRESCRIBED end-effector task  --  *** EDIT THIS BLOCK ***
%     Single source of truth: prescribed_task(tq, tp) evaluates p_e, R_e and
%     the headings (with analytic derivatives) and is used BOTH here and by
%     the online reference plan.ref.
% ========================================================================
[r0c0, r0e0] = arm_kin([0;0;0;0], params);
tp.T   = T;
tp.pe0 = params.hover.r_ed;                 % EE start = hover EE point

% --- EE position: net displacement + a mid-path arc, covering an x-y-z box.
%     Timed by the min-jerk phase sigma(t) so the motion is rest-to-rest
%     regardless of the shape; edit the spans/arcs for your workspace. -- EDIT
tp.Dx = 1.2;  tp.Dy = 0.6;  tp.Dz = 0.4;    % net EE displacement (m)
tp.Ay = 0.3;  tp.Az = 0.25;                 % lateral/vertical arc amplitudes (m)

% --- EE orientation sweep: this is what makes the JOINTS move.       -- EDIT
%     R_e = Rz(yaw) Ry(pitch) Rz(wrist).  yaw(-platform) drives q1,
%     pitch=beta drives q2,q3, wrist drives q4.
tp.yaw0 = deg2rad(0);   tp.yawA = deg2rad(70);   % EE heading yaw  0 -> 70 deg
tp.b0   = deg2rad(40);  tp.bA   = deg2rad(30);   % EE tilt beta (sin arc) 40..70..40 deg
                                                 %   KEEP beta in ~40-70 (split 0.5) for cond(J_3y)<~100
tp.g0   = deg2rad(0);   tp.gA   = deg2rad(55);   % EE wrist roll   0 -> 55 deg

% --- Platform heading, INDEPENDENT of the EE heading (need not align). -- EDIT
%     q1 ~ (EE yaw) - (platform yaw), so an opposite platform turn widens q1.
tp.pp0  = deg2rad(0);   tp.ppA  = deg2rad(-25);  % platform yaw 0 -> -25 deg

% --- Redundancy split (q2 = split*beta, q3 = (1-split)*beta) ---------- -- EDIT
tp.split = 0.5;                              % 0.5 = theta2=theta3; use 0.25/0.75 near beta~90

% precompute the prescribed signals on the recovery grid (zeroth order)
PE = zeros(3,opts.N); B1D = zeros(3,opts.N); RE = zeros(3,3,opts.N);
for k = 1:opts.N
    tk = prescribed_task(t(k), tp);
    PE(:,k) = tk.r_ed;  B1D(:,k) = tk.b1_d;  RE(:,:,k) = tk.R_e;
end

% ========================================================================
% 2.  RECOVER p_c  --  damped fixed-point on a polynomial p_c
% ========================================================================
tc = (t(1)+t(end))/2;  ts = (t(end)-t(1))/2;  tau = (t - tc)/ts;
Pc = fit_poly_cols(tau, PE, opts.deg);          % init: p_c ~ p_e
hist = nan(opts.maxit,1);
for it = 1:opts.maxit
    [~,~,pcdd] = eval_poly_cols(Pc, tau, ts);
    pc_new = zeros(3,opts.N);
    for k = 1:opts.N
        ac   = pcdd(:,k) + g*e3;
        tdir = ac/norm(ac);                      % (C1)
        R0   = build_R0(tdir, B1D(:,k));         % ctrl 98-101
        Re0  = R0.'*RE(:,:,k);
        [a,b,c] = zyz_angles(Re0);               % z-y-z
        q    = [a; tp.split*b; (1-tp.split)*b; c]; % (R2) elbow split
        [r0c,r0e] = arm_kin(q, params);
        pc_new(:,k) = PE(:,k) + R0*(r0c - r0e);  % (C2)
    end
    pc_cur = eval_poly_cols(Pc, tau, ts);
    Pc_new = fit_poly_cols(tau, (1-opts.relax)*pc_cur + opts.relax*pc_new, opts.deg);
    hist(it) = max(vecnorm(eval_poly_cols(Pc_new,tau,ts) - pc_cur, 2, 1));
    Pc = Pc_new;
    if hist(it) < opts.tol, break; end
end
nit = it;
[p_c, ~, p_c_d2] = eval_poly_cols(Pc, tau, ts);

% ========================================================================
% 3.  DIAGNOSTICS (fine grid) -- dynamic defect + regularity guards
% ========================================================================
tf = linspace(0,T,opts.Nfine).';  tauf = (tf-tc)/ts;
[pcf,~,pcddf] = eval_poly_cols(Pc, tauf, ts);
e_dyn = zeros(3,opts.Nfine); beta_f = zeros(1,opts.Nfine);
acn_f = zeros(1,opts.Nfine); tilt_f = zeros(1,opts.Nfine); th_f = zeros(4,opts.Nfine);
cond_f = zeros(1,opts.Nfine); sig_f = zeros(1,opts.Nfine);
for k = 1:opts.Nfine
    tk = prescribed_task(tf(k), tp);
    ac = pcddf(:,k) + g*e3;  tdir = ac/norm(ac);
    R0 = build_R0(tdir, tk.b1_d);  Re0 = R0.'*tk.R_e;
    [a,b,c] = zyz_angles(Re0);  q = [a; tp.split*b; (1-tp.split)*b; c];
    [r0c,r0e] = arm_kin(q, params);
    e_dyn(:,k) = pcf(:,k) - (tk.r_ed + R0*(r0c - r0e));
    beta_f(k) = b;  acn_f(k) = norm(ac);
    tilt_f(k) = acosd(min(1,max(-1, tdir.'*e3)));  th_f(:,k) = q;
    s_k = svd(J03y(q, params));            % arm task-Jacobian spectrum (controller inverts J_3y)
    cond_f(k) = s_k(1)/s_k(end);           % conditioning
    sig_f(k)  = s_k(end);                  % smallest singular value
end
diag.iterations      = nit;
diag.conv_history    = hist(1:nit);
diag.max_dyn_defect  = max(vecnorm(e_dyn,2,1));
diag.min_sin_beta    = min(abs(sin(beta_f)));
diag.beta_range_deg  = [min(beta_f) max(beta_f)]*180/pi;
diag.min_acc_norm    = min(acn_f);
diag.max_thrust_tilt = max(tilt_f);
diag.max_cond_J3y    = max(cond_f);     % <-- controller inverts J_3y; keep this modest
diag.cond_J3y_t0     = cond_f(1);
diag.min_sigma_J3y   = min(sig_f);      % smallest sigma_min over the trajectory
diag.sigma_J3y_t0    = sig_f(1);
diag.theta_min_deg   = min(th_f,[],2)*180/pi;
diag.theta_max_deg   = max(th_f,[],2)*180/pi;

fprintf('\n=== compatible-CoM planning ===\n');
fprintf('Picard iterations          : %d  (final step %.2e m)\n', nit, hist(nit));
fprintf('MAX dynamic defect |e_dyn| : %.3e m   <-- should be ~0\n', diag.max_dyn_defect);
fprintf('min |sin(beta)|            : %.3e     (~0 => wrist singularity)\n', diag.min_sin_beta);
fprintf('beta range                 : [%.1f, %.1f] deg\n', diag.beta_range_deg);
fprintf('max thrust tilt            : %.1f deg\n', diag.max_thrust_tilt);
fprintf('cond(J_3y^0): start %.0f | max %.0f | sigma_min: start %.2e | min %.2e   (geometric arm Jacobian)\n', ...
        diag.cond_J3y_t0, diag.max_cond_J3y, diag.sigma_J3y_t0, diag.min_sigma_J3y);
if diag.min_sin_beta < 1e-2
    fprintf(2,'WARNING: near WRIST singularity (beta~0) -- raise the EE tilt tp.b0/tp.bA.\n');
end
if diag.max_cond_J3y > 300
    fprintf(2,['WARNING: near ARM (elbow) singularity, cond(J_3y)=%.0f.\n' ...
               '         Keep beta (tp.b0/tp.bA) in 40-70 deg, or set tp.split to 0.25/0.75.\n'], diag.max_cond_J3y);
end

% ========================================================================
% 4.  PACKAGE: online reference handle + consistent initial state
% ========================================================================
plan.t = t; plan.p_e = PE; plan.p_c = p_c; plan.beta = beta_f;
plan.theta = th_f(:, round(linspace(1,opts.Nfine,opts.N)));
plan.Pc = Pc; plan.tc = tc; plan.ts = ts; plan.tp = tp;
plan.diag = diag; plan.opts = opts; plan.T = T;

% drop-in reference (cheap: polynomial + analytic trig)
plan.ref = @(tq) eval_full_ref(tq, Pc, tc, ts, tp);

% consistent t=0 configuration -> X0 with ZERO initial tracking error
[r0_0, R0_0, q0] = reconstruct_config(0, Pc, tc, ts, tp, params, g);
plan.r0_0 = r0_0;  plan.R0_0 = R0_0;  plan.q0 = q0;
plan.X0 = [ r0_0; R0_0(:); q0; zeros(3,1); zeros(3,1); zeros(params.n,1) ];

if opts.plot
    make_plots(t, PE, p_c, p_c_d2, tf, e_dyn, beta_f, th_f, tilt_f, hist(1:nit));
end
end % ===================== main =========================================


% ========================================================================
%  Prescribed task  (p_e, R_e, headings + analytic derivatives)
% ========================================================================
function task = prescribed_task(tq, P)
% Rich rest-to-rest task: 3-axis EE position, time-varying EE orientation
% (yaw/pitch/wrist) and an INDEPENDENT platform heading.  Everything is a
% function of the min-jerk phase sigma(t); time derivatives come from the
% chain rule with sigma_dot, sigma_ddot, so the start/stop are at rest.
[s, ds, d2s] = minjerk3(tq / P.T);
sg = s;  sd = ds / P.T;  sdd = d2s / P.T^2;        % sigma and its time derivs
Sz = [0 -1 0; 1 0 0; 0 0 0];   Sy = [0 0 1; 0 0 0; -1 0 0];   % hat(e3), hat(e2)
pic = pi;

% ---- EE position p_e(sigma): linear + sinusoidal arc; sigma-derivatives ----
X  = P.Dx*sg;                 Xs = P.Dx;                       Xss = 0;
Y  = P.Dy*sg + P.Ay*sin(pic*sg);  Ys = P.Dy + P.Ay*pic*cos(pic*sg);  Yss = -P.Ay*pic^2*sin(pic*sg);
Z  = P.Dz*sg + P.Az*sin(pic*sg);  Zs = P.Dz + P.Az*pic*cos(pic*sg);  Zss = -P.Az*pic^2*sin(pic*sg);
pe_s = [X;Y;Z];  pe_s1 = [Xs;Ys;Zs];  pe_s2 = [Xss;Yss;Zss];
task.r_ed      = P.pe0 + pe_s;
task.r_ed_dot  = pe_s1*sd;
task.r_ed_ddot = pe_s2*sd^2 + pe_s1*sdd;

% ---- EE orientation R_e = Rz(al) Ry(be) Rz(ga); sigma-derivatives by product rule ----
al = P.yaw0 + P.yawA*sg;            al1 = P.yawA;                  al2 = 0;
be = P.b0   + P.bA*sin(pic*sg);     be1 = P.bA*pic*cos(pic*sg);    be2 = -P.bA*pic^2*sin(pic*sg);
ga = P.g0   + P.gA*sg;              ga1 = P.gA;                    ga2 = 0;
A = Rz(al); B = Ry(be); C = Rz(ga);
dA = Sz*A*al1;  d2A = Sz*Sz*A*al1^2 + Sz*A*al2;
dB = Sy*B*be1;  d2B = Sy*Sy*B*be1^2 + Sy*B*be2;
dC = Sz*C*ga1;  d2C = Sz*Sz*C*ga1^2 + Sz*C*ga2;
Re   = A*B*C;
dRe  = dA*B*C + A*dB*C + A*B*dC;
d2Re = d2A*B*C + A*d2B*C + A*B*d2C + 2*(dA*dB*C + dA*B*dC + A*dB*dC);
task.R_e = Re;
b1 = Re(:,1);  b1_s1 = dRe(:,1);  b1_s2 = d2Re(:,1);     % b1_de and sigma-derivs
task.b1_de      = b1;
task.b1_de_dot  = b1_s1*sd;
task.b1_de_ddot = b1_s2*sd^2 + b1_s1*sdd;

% ---- platform heading b1_d(sigma), independent of the EE yaw ----
pp = P.pp0 + P.ppA*sg;   pp1 = P.ppA;   pp2 = 0;
b1d    = [cos(pp); sin(pp); 0];
b1d_s1 = pp1*[-sin(pp); cos(pp); 0];
b1d_s2 = pp2*[-sin(pp); cos(pp); 0] + pp1^2*[-cos(pp); -sin(pp); 0];
task.b1_d      = b1d;
task.b1_d_dot  = b1d_s1*sd;
task.b1_d_ddot = b1d_s2*sd^2 + b1d_s1*sdd;
end

% ========================================================================
%  Online FULL reference (drop-in for plan_locked_trajectory)
% ========================================================================
function ref = eval_full_ref(tq, Pc, tc, ts, tp)
tau = (tq - tc)/ts;
[p,p1,p2,p3,p4] = eval_poly_cols(Pc, tau, ts);
ref.x_cd = p; ref.x_cd_dot = p1; ref.x_cd_ddot = p2; ref.x_cd_d3 = p3; ref.x_cd_d4 = p4;
task = prescribed_task(tq, tp);
ref.b1_d  = task.b1_d;   ref.b1_d_dot  = task.b1_d_dot;   ref.b1_d_ddot  = task.b1_d_ddot;
ref.r_ed  = task.r_ed;   ref.r_ed_dot  = task.r_ed_dot;   ref.r_ed_ddot  = task.r_ed_ddot;
ref.b1_de = task.b1_de;  ref.b1_de_dot = task.b1_de_dot;  ref.b1_de_ddot = task.b1_de_ddot;
end

% ========================================================================
%  Consistent configuration (r_0, R_0, q) at a query time  -> for X0
% ========================================================================
function [r_0, R_0, q] = reconstruct_config(tq, Pc, tc, ts, tp, params, g)
tau = (tq - tc)/ts;
[~,~,pcdd] = eval_poly_cols(Pc, tau, ts);
task = prescribed_task(tq, tp);
ac = pcdd + g*[0;0;1];  tdir = ac/norm(ac);
R_0 = build_R0(tdir, task.b1_d);
[a,b,c] = zyz_angles(R_0.'*task.R_e);
q = [a; tp.split*b; (1-tp.split)*b; c];
[~, r0e] = arm_kin(q, params);
r_0 = task.r_ed - R_0*r0e;                 % base pos s.t. EE matches r_ed
end


% ========================================================================
%  Coupling / kinematics helpers
% ========================================================================
function R0 = build_R0(t_dir, b1_d)
b3c = t_dir/norm(t_dir);  P1 = eye(3) - b3c*b3c.';  s = P1*b1_d;  ns = norm(s);
if ns < 1e-9
    tmp = [1;0;0]; if abs(b3c(1))>0.9, tmp=[0;1;0]; end
    s = (eye(3)-b3c*b3c.')*tmp; ns = norm(s);
end
b1c = s/ns;  b2c = cross(b3c,b1c);  R0 = [b1c, b2c, b3c];
end

function [a,b,c] = zyz_angles(M)
sb = hypot(M(1,3), M(2,3));  b = atan2(sb, M(3,3));
if sb > 1e-9
    a = atan2(M(2,3), M(1,3));  c = atan2(M(3,2), -M(3,1));
else
    a = atan2(-M(1,2), M(1,1)); c = 0;
end
end

function [r0c, r0e] = arm_kin(q, params)
n = params.n; mi = params.m_i; li = params.l_i; h = params.h_i_im1; mT = sum(mi);
Ri0 = repmat(eye(3),1,1,n+1);
for i = 1:n, Ri0(:,:,i+1) = Ri0(:,:,i)*elem_rot(h(:,i), q(i)); end
rO = zeros(3,n+1);
for i = 1:n, rO(:,i+1) = rO(:,i) + Ri0(:,:,i+1)*li(:,i); end
r0e = rO(:,n+1);
rcom = zeros(3,1);
for i = 1:n, rcom = rcom + mi(i+1)*(rO(:,i) + Ri0(:,:,i+1)*(li(:,i)/2)); end
r0c = rcom/mT;
end

function J0 = J03y(q, params)
% Arm-only end-effector task Jacobian J_3y^0 (3 EE-pos rows + 1 EE-yaw row)
% in {0}.  This is the matrix the controller inverts (u3 = J_3y \\ inner);
% its conditioning is the relevant arm-singularity measure.  (PDF2 Remark 15)
n = params.n; h = params.h_i_im1; li = params.l_i;
Ri = repmat(eye(3),1,1,n+1);
for i = 1:n, Ri(:,:,i+1) = Ri(:,:,i)*elem_rot(h(:,i), q(i)); end
h0 = zeros(3,n); for i = 1:n, h0(:,i) = Ri(:,:,i)*h(:,i); end      % h_k^0
O  = zeros(3,n+1); for i = 1:n, O(:,i+1) = O(:,i) + Ri(:,:,i+1)*li(:,i); end
r0e = O(:,n+1); b3e = Ri(:,:,n+1)*[0;0;1];
Jp = zeros(3,n); yaw = zeros(1,n);
for k = 1:n, Jp(:,k) = cross(h0(:,k), r0e-O(:,k)); yaw(k) = b3e.'*h0(:,k); end
J0 = [Jp; yaw];
end

function R = elem_rot(h, q)
H = [0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0];
R = eye(3) + sin(q)*H + (1-cos(q))*(H*H);
end
function R = Rz(a), R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end
function R = Ry(a), R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end


% ========================================================================
%  Polynomial fit / evaluation (scaled monomial basis, base MATLAB)
% ========================================================================
function P = fit_poly_cols(tau, Y, deg)
P = struct('c', cell(1,3));
for i = 1:3, P(i).c = polyfit(tau(:), Y(i,:).', deg); end
end

function [p, p1, p2, p3, p4] = eval_poly_cols(P, tau, ts)
N = numel(tau); p = zeros(3,N); p1=p; p2=p; p3=p; p4=p;
for i = 1:3
    c = P(i).c; c1=polyder(c); c2=polyder(c1); c3=polyder(c2); c4=polyder(c3);
    p(i,:)  = polyval(c , tau);
    p1(i,:) = polyval(c1, tau)/ts;
    p2(i,:) = polyval(c2, tau)/ts^2;
    p3(i,:) = polyval(c3, tau)/ts^3;
    p4(i,:) = polyval(c4, tau)/ts^4;
end
end


% ========================================================================
%  Minimum-jerk profile and derivatives, u in [0,1]
% ========================================================================
function [s, ds, d2s] = minjerk3(u)
u = min(1, max(0, u));
s   = 10*u.^3 - 15*u.^4 + 6*u.^5;
ds  = 30*u.^2 - 60*u.^3 + 30*u.^4;
d2s = 60*u    - 180*u.^2 + 120*u.^3;
end


% ========================================================================
%  Default model parameters  (mirrors main_sim.m, modified config)
% ========================================================================
function params = default_params()
n = 4; params.n = n; params.g = 9.81;
params.m_i = [1.7, 0.105, 0.143, 0.135, 0.236];
L = [0.077, 0.130, 0.124, 0.126]; params.L = L;
h = zeros(3,n); h(:,1)=[0;0;1]; h(:,2)=[0;1;0]; h(:,3)=[0;1;0]; h(:,4)=[0;0;1];
params.h_i_im1 = h;
l = zeros(3,n);
l(:,1)=[0;0;-L(1)]; l(:,2)=[0;0;-L(2)]; l(:,3)=[L(3);0;0]; l(:,4)=[0;0;-L(4)];
params.l_i = l;
r_0 = [0;0;0.5];  [r0c,r0e] = arm_kin([0;0;0;0], params);
params.hover.x_cd = r_0 + r0c;  params.hover.r_ed = r_0 + r0e;
params.hover.b1_d = [1;0;0];
end


% ========================================================================
%  utilities + plotting
% ========================================================================
function s = setdef(s, f, v)
if ~isfield(s,f) || isempty(s.(f)), s.(f) = v; end
end

function make_plots(t, p_e, p_c, p_c_d2, tf, e_dyn, beta_f, th_f, tilt_f, hist)
figure('Name','compatible-CoM plan','Color','w','Position',[80 80 1100 720]);
subplot(2,3,1); hold on; grid on;
plot(t, p_c.','LineWidth',1.5); set(gca,'ColorOrderIndex',1);
plot(t, p_e.','--','LineWidth',1.0);
xlabel('t (s)'); ylabel('m'); title('p_c (solid) vs p_e (dashed)');
legend({'p_{c,x}','p_{c,y}','p_{c,z}'},'Location','best');
subplot(2,3,2); grid on; plot(t, p_c_d2.','LineWidth',1.2);
xlabel('t (s)'); ylabel('m/s^2'); title('p_c'''' (analytic)'); legend({'x','y','z'});
subplot(2,3,3); grid on; semilogy(1:numel(hist), max(hist,1e-16),'o-','LineWidth',1.2);
xlabel('Picard iteration'); ylabel('max \Delta p_c (m)'); title('convergence');
subplot(2,3,4); grid on; plot(tf, vecnorm(e_dyn,2,1),'LineWidth',1.4);
xlabel('t (s)'); ylabel('|e_{dyn}| (m)'); title('dynamic defect');
subplot(2,3,5); hold on; grid on; plot(tf, th_f.'*180/pi,'LineWidth',1.2);
xlabel('t (s)'); ylabel('deg'); title('recovered \theta');
legend({'\theta_1','\theta_2','\theta_3','\theta_4'},'Location','best');
subplot(2,3,6); hold on; grid on;
yyaxis left;  plot(tf, beta_f*180/pi,'LineWidth',1.4); ylabel('\beta (deg)');
yyaxis right; plot(tf, tilt_f,'LineWidth',1.2); ylabel('thrust tilt (deg)');
xlabel('t (s)'); title('\beta and thrust tilt');
end