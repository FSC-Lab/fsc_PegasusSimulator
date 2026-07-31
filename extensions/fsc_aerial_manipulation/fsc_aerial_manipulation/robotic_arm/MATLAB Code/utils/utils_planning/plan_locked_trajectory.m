function ref = plan_locked_trajectory(t, params)
%PLAN_LOCKED_TRAJECTORY  Reference trajectory with the end-effector LOCKED to
%   the platform (arm holds its initial joint configuration) for the aerial
%   manipulator.
%
%   ref = plan_locked_trajectory(t, params) returns the reference signals
%   required by the controller flowchart:
%
%       System CoM:   x_cd, x_cd_dot, x_cd_ddot, x_cd_d3, x_cd_d4
%       Base heading: b1_d,  b1_d_dot,  b1_d_ddot
%       End-effector: r_ed,  r_ed_dot,  r_ed_ddot
%       EE heading:   b1_de, b1_de_dot, b1_de_ddot
%
%   ALL trajectory parameters now live in THIS file, in the local function
%   traj_config() (search for "SELECT THE REFERENCE TRAJECTORY").  main_sim
%   no longer configures the trajectory; it only supplies the initial-state
%   ANCHORS through params.hover (initial system CoM, EE offset, heading),
%   which are combined with the selected config in build_traj() on every call.
%
%   CONFIG QUERY:  plan_locked_trajectory('config', params)  (params optional)
%   returns the config struct (with fields .type and .T) so callers such as
%   main_sim (simulation horizon) and visualize_reference can read the active
%   trajectory type and duration without running dynamics.
%
%   Every trajectory is time-scaled by a minimum-jerk profile so it starts
%   (and ends) AT REST: position, velocity, acceleration, heading AND yaw-rate
%   all match the initial state at t=0 (zero initial tracking error).  In every
%   case the END-EFFECTOR is held FIXED RELATIVE TO THE PLATFORM: the EE-to-CoM
%   offset d0 is carried with the base frame, so the arm holds its initial
%   (non-singular) joint configuration.
%
%   Supported trajectory types (cfg.type):
%     'hover'  : hold the initial pose for T seconds (all derivatives zero).
%     'line'   : straight line, distance D along a SELECTABLE axis direction
%                (x / y / z / xy / xz / yz / xyz); constant heading, so the EE
%                offset is constant:  r_ed = x_cd + d0.
%     'sine'   : sinusoidal oscillation (amplitude A, Ncyc cycles) along a
%                SELECTABLE axis direction; rest-to-rest via min-jerk phase
%                (zero initial velocity/accel), constant heading, constant EE
%                offset:  r_ed = x_cd + d0.
%     'circle' : ONE revolution of radius r in the x-y plane; the heading
%                follows the tangent (the base yaws), so the EE offset rides
%                WITH the yaw:  r_ed = x_cd + Rz(phi) d0,  phi = theta - theta0
%                (arm still fixed relative to the platform).
%     'poly'   : gentle polynomial lane-change S-curve, POSITION ONLY; heading
%                held FIXED at the initial heading, constant EE offset.
%
%   For 'line' / 'sine' the axis direction is built from a short axis string:
%       'x' 'y' 'z'           -> single axis
%       'xy' 'xz' 'yz' 'xyz'  -> equal-weight diagonal of those axes
%
%   Requires on the path:  (none beyond this file).

% ---- all tunable trajectory parameters are defined here -----------------
cfg = traj_config();

% ---- optional active-type override from the caller ----------------------
%   main_sim selects the locked shape via params.traj_type ('hover' | 'line' |
%   'sine' | 'circle' | 'poly'); per-type tuning still lives in traj_config().
%   Absent -> the cfg.type set inside traj_config() is used.
if (nargin >= 2) && isstruct(params) && isfield(params,'traj_type') ...
        && ~isempty(params.traj_type)
    cfg.type = char(params.traj_type);
    if ~isfield(cfg, cfg.type)
        error('plan_locked_trajectory:type', ...
              'Unknown trajectory type "%s" (params.traj_type).', cfg.type);
    end
    cfg.T = cfg.(cfg.type).T;                 % re-resolve the active duration
end

% ---- config-query mode:  plan_locked_trajectory('config'[, params]) ---------
if ischar(t) || isstring(t)
    ref = cfg;                 % return config (has .type and .T)
    return;
end

% ---- assemble the geometric trajectory from config + initial anchors ----
tr = build_traj(cfg, params);

switch tr.type
    case 'hover',  ref = traj_hover (t, tr);
    case 'line',   ref = traj_line  (t, tr);
    case 'sine',   ref = traj_sine  (t, tr);
    case 'circle', ref = traj_circle(t, tr);
    case 'poly',   ref = traj_poly  (t, tr);
    otherwise
        error('plan_locked_trajectory:type', ...
              'Unknown trajectory type "%s".', tr.type);
end
end

% ========================================================================
%  traj_config  --  *** SELECT THE REFERENCE TRAJECTORY AND ITS PARAMS ***
%       Edit cfg.type to pick the active trajectory; edit the matching block
%       to tune it.  cfg.T is resolved to the active type's duration.
% ========================================================================
function cfg = traj_config()

% ===================== SELECT THE REFERENCE TRAJECTORY ===================
cfg.type = 'hover';     % 'hover' | 'line' | 'sine' | 'circle' | 'poly'
% =========================================================================

% ---- hover: hold the initial pose -----------------------------------
cfg.hover.T    = 5.0;        % hold duration (s)                     -- TUNE

% ---- line: straight segment along an axis direction -----------------
cfg.line.axis  = 'xy';        % 'x'|'y'|'z'|'xy'|'xz'|'yz'|'xyz'      -- TUNE
cfg.line.D     = 2.0;        % travel distance (m)                   -- TUNE
cfg.line.T     = 10;         % maneuver duration (s)                 -- TUNE

% ---- sine: sinusoid along an axis direction -------------------------
cfg.sine.axis  = 'xy';        % 'x'|'y'|'z'|'xy'|'xz'|'yz'|'xyz'      -- TUNE
cfg.sine.A     = 0.5;        % amplitude (m)                         -- TUNE
cfg.sine.Ncyc  = 2;          % number of full sine cycles over [0,T] -- TUNE
cfg.sine.T     = 12;         % maneuver duration (s)                 -- TUNE

% ---- circle: one rest-to-rest revolution in the x-y plane -----------
cfg.circle.r   = 1.0;        % radius (m)                            -- TUNE
cfg.circle.T   = 20;         % time for ONE revolution (s)           -- TUNE

% ---- poly: gentle lane-change S-curve -------------------------------
cfg.poly.D     = 2.0;        % forward distance (m)                  -- TUNE
cfg.poly.A     = 0.2;        % lateral excursion (m)                 -- TUNE
cfg.poly.T     = 12;         % maneuver duration (s)                 -- TUNE

% --- resolve the active duration (used as the simulation horizon) --------
if ~isfield(cfg, cfg.type)
    error('plan_locked_trajectory:type', ...
          'Unknown trajectory type "%s" in traj_config.', cfg.type);
end
active = cfg.(cfg.type);
cfg.T  = active.T;
end

% ========================================================================
%  build_traj  --  combine the selected config with the initial-state
%       ANCHORS (params.hover) into the geometric struct tr consumed by the
%       traj_* functions.  Anchors:
%         x_c0 = initial system CoM,  d0 = constant EE-to-CoM offset,
%         b1_0 = initial base heading (all in {I}).
% ========================================================================
function tr = build_traj(cfg, params)
x_c0 = params.hover.x_cd;
d0   = params.hover.r_ed - params.hover.x_cd;
b1_0 = params.hover.b1_d;

tr      = struct();
tr.type = cfg.type;
tr.x_c0 = x_c0;
tr.d0   = d0;

switch cfg.type
    case 'hover'
        tr.T  = cfg.hover.T;
        tr.b1 = b1_0;

    case 'line'
        tr.dir = axis_dir(cfg.line.axis);
        tr.D   = cfg.line.D;
        tr.T   = cfg.line.T;
        tr.b1  = b1_0;

    case 'sine'
        tr.e    = axis_dir(cfg.sine.axis);
        tr.A    = cfg.sine.A;
        tr.Ncyc = cfg.sine.Ncyc;
        tr.T    = cfg.sine.T;
        tr.b1   = b1_0;

    case 'circle'
        tr.r = cfg.circle.r;
        tr.T = cfg.circle.T;
        % phase so the initial tangent equals the initial heading (no yaw jump)
        hxy = [b1_0(1); b1_0(2); 0];
        if norm(hxy) < 1e-9
            tr.theta0 = 0;                          % heading ~ vertical: fall back
        else
            hxy = hxy / norm(hxy);
            tr.theta0 = atan2(-hxy(1), hxy(2));     % tangent(theta0) = hxy
        end
        % center so x_cd(0) = x_c0; circle lies in the plane z = x_c0(3)
        tr.c    = x_c0 - tr.r*[cos(tr.theta0); sin(tr.theta0); 0];
        tr.c(3) = x_c0(3);

    case 'poly'
        tr.D = cfg.poly.D;
        tr.A = cfg.poly.A;
        tr.T = cfg.poly.T;
        ef = [b1_0(1); b1_0(2); 0];                 % forward (horizontal heading)
        if norm(ef) < 1e-9, ef = [1;0;0]; end
        ef = ef / norm(ef);
        tr.e_fwd = ef;                              % forward unit vector in {I}
        tr.e_lat = [-ef(2); ef(1); 0];              % left = z x e_fwd  (unit)
        tr.b1_0  = ef;                              % initial heading (for EE yaw ref)

    otherwise
        error('plan_locked_trajectory:type', ...
              'Unknown trajectory type "%s".', cfg.type);
end
end

% ========================================================================
%  axis_dir  --  unit direction from a short axis string
%       'x' 'y' 'z'           -> single axis
%       'xy' 'xz' 'yz' 'xyz'  -> equal-weight diagonal of those axes
% ========================================================================
function e = axis_dir(s)
switch lower(string(s))
    case "x",   e = [1;0;0];
    case "y",   e = [0;1;0];
    case "z",   e = [0;0;1];
    case "xy",  e = [1;1;0];
    case "xz",  e = [1;0;1];
    case "yz",  e = [0;1;1];
    case "xyz", e = [1;1;1];
    otherwise
        error('plan_locked_trajectory:axis', ...
              'Unknown axis "%s" (use x|y|z|xy|xz|yz|xyz).', char(s));
end
e = e / norm(e);
end

% ========================================================================
%  minjerk  --  rest-to-rest minimum-jerk scalar profile and derivatives
%       s(tau), tau in [0,1];  s,ds,d2s = 0 at both ends.  Derivatives are
%       w.r.t. tau (apply chain rule 1/T^k for d^k/dt^k).  Clamped outside.
% ========================================================================
function [s, ds, d2s, d3s, d4s] = minjerk(tau)
if tau <= 0
    s = 0;  ds = 0;  d2s = 0;  d3s = 0;  d4s = 0;
elseif tau >= 1
    s = 1;  ds = 0;  d2s = 0;  d3s = 0;  d4s = 0;
else
    s   =  10*tau^3 - 15*tau^4 +  6*tau^5;
    ds  =  30*tau^2 - 60*tau^3 + 30*tau^4;
    d2s =  60*tau   -180*tau^2 +120*tau^3;
    d3s =  60       -360*tau   +360*tau^2;
    d4s = -360      +720*tau;
end
end

% ========================================================================
%  set_fixed_ee_heading  --  shared helper for the trajectories that keep the
%       end-effector at a CONSTANT offset d0 and a CONSTANT (shared) heading.
%       Fills r_ed (= x_cd + d0) and the base/EE headings (= b1, zero rates).
% ========================================================================
function ref = set_fixed_ee_heading(ref, tr)
ref.r_ed      = ref.x_cd      + tr.d0;
ref.r_ed_dot  = ref.x_cd_dot;
ref.r_ed_ddot = ref.x_cd_ddot;

z3 = zeros(3,1);
ref.b1_d      = tr.b1;   ref.b1_d_dot  = z3;   ref.b1_d_ddot  = z3;
ref.b1_de     = tr.b1;   ref.b1_de_dot = z3;   ref.b1_de_ddot = z3;
% ref.b1_de     = [cos(pi/6);sin(pi/6);0];   ref.b1_de_dot = z3;   ref.b1_de_ddot = z3;
end

% ========================================================================
%  traj_hover  --  hold the initial pose (all CoM derivatives zero)
% ========================================================================
function ref = traj_hover(t, tr) %#ok<INUSL>
z3 = zeros(3,1);
ref.x_cd      = tr.x_c0;
ref.x_cd_dot  = z3;
ref.x_cd_ddot = z3;
ref.x_cd_d3   = z3;
ref.x_cd_d4   = z3;

ref = set_fixed_ee_heading(ref, tr);
end

% ========================================================================
%  traj_line  --  straight line, smooth rest-to-rest, constant heading
%       Direction tr.dir (unit) is built from the axis selection.
% ========================================================================
function ref = traj_line(t, tr)
T = tr.T;   D = tr.D;   e = tr.dir;
[s, ds, d2s, d3s, d4s] = minjerk(t / T);

% --- CoM: straight line along e, smooth start/stop ---
ref.x_cd      = tr.x_c0 + D*s        * e;
ref.x_cd_dot  =           D*ds /T    * e;
ref.x_cd_ddot =           D*d2s/T^2  * e;
ref.x_cd_d3   =           D*d3s/T^3  * e;
ref.x_cd_d4   =           D*d4s/T^4  * e;

% --- EE: constant offset + constant shared heading (arm fixed to platform) ---
ref = set_fixed_ee_heading(ref, tr);
end

% ========================================================================
%  traj_sine  --  sinusoidal oscillation along a fixed axis direction.
%       Geometry:  x_cd(t) = x_c0 + A sin(p(t)) e,  e a unit direction.
%       Phase:     p(t) = 2*pi*Ncyc * s(t/T), with s the min-jerk profile,
%                  so the motion eases on/off from REST (zero initial velocity,
%                  acceleration AND a return to x_c0 at t=T -- Ncyc full sine
%                  oscillations).  Time derivatives via Faa di Bruno through
%                  p(t).  Constant heading + constant EE offset, as in 'line'.
% ========================================================================
function ref = traj_sine(t, tr)
T = tr.T;   A = tr.A;   e = tr.e;   K = 2*pi*tr.Ncyc;
[s, ds, d2s, d3s, d4s] = minjerk(t / T);

% phase p(t) = K*s and its time derivatives (chain rule 1/T^k)
p  = K*s;
p1 = K*ds  / T;
p2 = K*d2s / T^2;
p3 = K*d3s / T^3;
p4 = K*d4s / T^4;
sp = sin(p);   cp = cos(p);

% time derivatives of f(t) = sin(p(t))  (Faa di Bruno)
f0 = sp;
f1 = cp*p1;
f2 = -sp*p1^2 + cp*p2;
f3 = -cp*p1^3 - 3*sp*p1*p2 + cp*p3;
f4 =  sp*p1^4 - 6*cp*p1^2*p2 - 3*sp*p2^2 - 4*sp*p1*p3 + cp*p4;

ref.x_cd      = tr.x_c0 + A*f0 * e;
ref.x_cd_dot  =           A*f1 * e;
ref.x_cd_ddot =           A*f2 * e;
ref.x_cd_d3   =           A*f3 * e;
ref.x_cd_d4   =           A*f4 * e;

% --- EE: constant offset + constant shared heading ---
ref = set_fixed_ee_heading(ref, tr);
end

% ========================================================================
%  traj_circle  --  ONE smooth (rest-to-rest) revolution in the x-y plane.
%       The phase theta(t) = theta0 + 2*pi*s(t/T) is time-scaled by the
%       minimum-jerk profile, so the drone eases onto/off the circle and the
%       initial velocity, acceleration and yaw rate are all zero.  The EE
%       offset is carried with the base yaw so the arm stays fixed.
% ========================================================================
function ref = traj_circle(t, tr)
r = tr.r;   c = tr.c;   T = tr.T;   th0 = tr.theta0;

% --- smooth phase theta(t) and its time derivatives (chain rule 1/T^k) ---
[s, ds, d2s, d3s, d4s] = minjerk(t / T);
th = th0 + 2*pi*s;
a  = 2*pi*ds  / T;        % theta'
b  = 2*pi*d2s / T^2;      % theta''
cc = 2*pi*d3s / T^3;      % theta'''
dd = 2*pi*d4s / T^4;      % theta''''

ct = cos(th);   st = sin(th);

% geometric derivatives of p(theta) = c + r [cos; sin; 0]
pth1 = r*[ -st;  ct; 0 ];     % dp/dtheta
pth2 = r*[ -ct; -st; 0 ];     % d2p/dtheta2
pth3 = r*[  st; -ct; 0 ];     % d3p/dtheta3
pth4 = r*[  ct;  st; 0 ];     % d4p/dtheta4

% --- 1. CoM position + time derivatives (Faa di Bruno through theta(t)) ---
ref.x_cd      = c + r*[ ct; st; 0 ];
ref.x_cd_dot  = pth1*a;
ref.x_cd_ddot = pth2*a^2 + pth1*b;
ref.x_cd_d3   = pth3*a^3 + 3*pth2*a*b + pth1*cc;
ref.x_cd_d4   = pth4*a^4 + 6*pth3*a^2*b + pth2*(3*b^2 + 4*a*cc) + pth1*dd;

% --- 2. EE: offset carried WITH the base yaw  phi = theta - theta0 --------
%        (phi' = a, phi'' = b), so the arm holds a fixed pose vs the platform.
phi = th - th0;   cph = cos(phi);   sph = sin(phi);
Rz  = [ cph, -sph, 0 ;                      % yaw about inertial vertical
        sph,  cph, 0 ;
          0,    0, 1 ];
Sz  = [ 0, -1, 0 ;                          % hat([0;0;1])
        1,  0, 0 ;
        0,  0, 0 ];
d_off = Rz * tr.d0;
ref.r_ed      = ref.x_cd      +            d_off;
ref.r_ed_dot  = ref.x_cd_dot  + a       *  Sz       * d_off;
ref.r_ed_ddot = ref.x_cd_ddot + b       *  Sz       * d_off ...
                              + a^2     * (Sz*Sz)    * d_off;

% --- 3. Headings: shared unit tangent of the circle (base = EE) -----------
b1   = [ -st;  ct; 0 ];        % unit tangent
db1  = [ -ct; -st; 0 ];        % d b1 / d theta
d2b1 = [  st; -ct; 0 ];        % d2 b1 / d theta2
ref.b1_d      = b1;
ref.b1_d_dot  = db1*a;
ref.b1_d_ddot = d2b1*a^2 + db1*b;

ref.b1_de      = b1;
ref.b1_de_dot  = db1*a;
ref.b1_de_ddot = d2b1*a^2 + db1*b;
end

% ========================================================================
%  traj_poly  --  gentle polynomial lane-change S-curve, POSITION ONLY.
%       Geometry:  P(sigma) = x_c0 + X(sigma) e_fwd + Y(sigma) e_lat,
%                  X = D*sigma,  Y = A*(3 sigma^2 - 2 sigma^3)   (sigma in [0,1]).
%       Timing:    sigma(t) = s(t/T), the min-jerk profile (rest-to-rest).
%       Heading:   FIXED at the initial heading b1_0 (ZERO yaw / yaw-rate /
%                  yaw-accel) -- the base does NOT turn to follow the path.
%       EE:        CONSTANT inertial offset d0 from the CoM (no rotation), so
%                  the EE is fixed RELATIVE TO THE CoM exactly as in traj_line.
% ========================================================================
function ref = traj_poly(t, tr)
T = tr.T;   D = tr.D;   A = tr.A;
ef = tr.e_fwd;   el = tr.e_lat;   b1_0 = tr.b1_0;

% --- min-jerk time scaling sigma(t) and its time derivatives -------------
[s, ds, d2s, d3s, d4s] = minjerk(t / T);
sig = s;   sd = ds/T;   sdd = d2s/T^2;   s3 = d3s/T^3;   s4 = d4s/T^4;

% --- geometric path P(sigma) and sigma-derivatives (in {I}) --------------
%     X = D*sigma (linear), Y = A(3 sigma^2 - 2 sigma^3) (cubic) => P'''' = 0.
X   = D*sig;                  Xp = D;   Xpp = 0;   Xppp = 0;
Y   = A*(3*sig^2 - 2*sig^3);  Yp = A*(6*sig - 6*sig^2);  Ypp = A*(6 - 12*sig);  Yppp = -12*A;
P   = tr.x_c0 + X*ef   + Y*el;
P1  =            Xp*ef  + Yp*el;       % dP/dsigma
P2  =            Xpp*ef + Ypp*el;      % d2P/dsigma2
P3  =            Xppp*ef+ Yppp*el;     % d3P/dsigma3

% --- 1. CoM position + time derivatives (chain rule through sigma(t)) ----
ref.x_cd      = P;
ref.x_cd_dot  = P1*sd;
ref.x_cd_ddot = P2*sd^2 + P1*sdd;
ref.x_cd_d3   = P3*sd^3 + 3*P2*sd*sdd + P1*s3;
ref.x_cd_d4   = 6*P3*sd^2*sdd + P2*(3*sdd^2 + 4*sd*s3) + P1*s4;   % P4 = 0

% --- 2. EE: CONSTANT offset from CoM; --- 3. Headings: CONSTANT b1_0 ------
%   (build tr.b1 from the poly heading so the shared helper can be reused)
tr.b1 = b1_0;
ref   = set_fixed_ee_heading(ref, tr);
end