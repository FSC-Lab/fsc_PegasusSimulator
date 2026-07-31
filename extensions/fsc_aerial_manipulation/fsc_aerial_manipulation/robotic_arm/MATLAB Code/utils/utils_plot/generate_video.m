function generate_video(log, params)
%GENERATE_VIDEO  Animate the closed-loop 3D scene (Figure 4 of plot_result)
%   and write it to a video file under ./video/.
%
%   generate_video(log, params) replays the recorded run in `log` and renders,
%   frame by frame, the SAME 3D picture that plot_result draws as its Figure 4:
%   the base / system-CoM / end-effector trajectories together with the full
%   aerial-manipulator body at the current pose.  In the 'disturbance' case the
%   transported payload is drawn as a small cube hanging at the end-effector.
%
%   The file is named with the trajectory type and the mode so different runs
%   do not overwrite each other, e.g.  video/trajectory3d_line_disturbance.mp4
%   (falls back to a Motion-JPEG .avi if the MPEG-4 profile is unavailable).
%
%   INPUT  log, params : exactly as produced/used in main_sim (see plot_result).
%
%   Requires on the path:  dynamics, plan_locked_trajectory, hat.

n = params.n;
t = log.t(:).';
N = numel(t);

% ---------------- Run descriptors (mode, trajectory type) ----------------
mode = 'nominal';
if isfield(params,'mode') && ~isempty(params.mode), mode = lower(params.mode); end
is_payload = strcmpi(mode, 'payload');
if isfield(params,'traj_tag') && ~isempty(params.traj_tag)
    ttype = params.traj_tag;                        % set by main_sim (both modes)
else
    try
        cfg   = plan_locked_trajectory('config', params);
        ttype = cfg.type;
    catch
        ttype = 'traj';
    end
end
run_tag = sprintf('%s_%s', ttype, mode);

% ---------------- Output folder + frame plan -----------------------------
vid_dir = 'video';
if ~exist(vid_dir, 'dir'), mkdir(vid_dir); end

fps     = 30;                                            % playback rate
dur     = max(t(end) - t(1), eps);
nframes = min(N, max(2, round(dur * fps)));              % ~ real-time, capped by N
nframes = min(nframes, 600);                             % hard cap on file size
idx     = unique(round(linspace(1, N, nframes)));

% ---------------- Open the video writer (mp4, else avi) ------------------
base = fullfile(vid_dir, sprintf('trajectory3d_%s', run_tag));
try
    outfile = [base '.mp4'];
    vw = VideoWriter(outfile, 'MPEG-4');
    vw.Quality = 95;
catch
    outfile = [base '.avi'];
    vw = VideoWriter(outfile, 'Motion JPEG AVI');
    vw.Quality = 95;
end
vw.FrameRate = fps;
open(vw);

% ---------------- Fixed scene limits (so the view never jumps) -----------
P = [log.r0, log.rc, log.re];                            % 3 x N all key points
ctr = 0.5*(max(P,[],2) + min(P,[],2));
half = 0.5*max(max(P,[],2) - min(P,[],2));
half = max(half, 0.1) + 0.55;                            % margin for quad + arm span
xl = ctr(1) + half*[-1 1];
yl = ctr(2) + half*[-1 1];
zl = ctr(3) + half*[-1 1];
arr_len = 0.30 * half;                                   % ground heading-arrow length (m)

if is_payload, s_cube = payload_cube_size(params); end

% ---------------- Figure (kept off-screen for clean capture) -------------
fig = figure('Color','w', 'Name','3D Trajectory (video)', ...
             'Position',[120 40 980 800]);
ax  = axes('Parent', fig);

fprintf('Rendering video (%d frames) ...\n', numel(idx));
for f = 1:numel(idx)
    k = idx(f);
    cla(ax); hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    % --- trajectory traces up to the current step ---
    plot3(ax, log.r0(1,1:k), log.r0(2,1:k), log.r0(3,1:k), '-',  'Color',[0.20 0.20 0.20], 'LineWidth',1.6);
    plot3(ax, log.rc(1,1:k), log.rc(2,1:k), log.rc(3,1:k), '--', 'Color',[0.10 0.65 0.20], 'LineWidth',1.6);
    plot3(ax, log.re(1,1:k), log.re(2,1:k), log.re(3,1:k), '-',  'Color',[0.85 0.45 0.05], 'LineWidth',1.6);

    % --- manipulator body at the current state ---
    [~, kin_k] = dynamics(log.X(:,k), params);
    draw_body(ax, kin_k, params);

    % --- current-pose rotations (orthonormalized), shared below -----------
    R0k   = reshape(log.X(4:12,k), 3, 3);
    [Uo,~,Vo] = svd(R0k);  R0k = Uo*Vo';
    R_e_k = R0k * kin_k.R_e_0;

    % --- payload cube hanging at the EE ('payload' mode only) ---
    if is_payload
        c = log.re(:,k) + [0;0;-s_cube/2];
        draw_payload_cube(ax, c, R_e_k, s_cube);
    end

    % --- heading arrows projected onto the ground plane z = zl(1) ----------
    %     Drop the drone CoM and the end-effector to the floor, then draw the
    %     x-y projection of each body-x heading from its foot point.
    %       platform heading  -> BLACK,   end-effector heading -> RED.
    hQ = ground_heading_arrow(ax, log.rc(:,k), R0k(:,1),   zl(1), arr_len, [0 0 0]);
    hE = ground_heading_arrow(ax, log.re(:,k), R_e_k(:,1), zl(1), arr_len, [0.85 0 0]);
    if ~isempty(hQ) && ~isempty(hE)
        legend(ax, [hQ hE], {'platform heading (x-y proj.)','EE heading (x-y proj.)'}, ...
               'Location','northeast','AutoUpdate','off','FontSize',9);
    end

    % --- fixed framing / labels / title ---
    axis(ax,'manual'); daspect(ax,[1 1 1]);
    xlim(ax,xl); ylim(ax,yl); zlim(ax,zl);
    view(ax,60,15); set(ax,'FontSize',10);  % Azimuth, Elevation
    xlabel(ax,'$x_I$ (m)','Interpreter','latex');
    ylabel(ax,'$y_I$ (m)','Interpreter','latex');
    zlabel(ax,'$z_I$ (m)','Interpreter','latex');
    title(ax, sprintf('Closed-loop 3D trajectory  (%s, %s)   t = %.2f s', ...
          ttype, mode, t(k)), 'Interpreter','none','FontWeight','bold');

    drawnow;
    writeVideo(vw, getframe(fig));
end

close(vw);
if ishandle(fig), close(fig); end
fprintf('Saved video: ./%s\n', outfile);
end

% ========================================================================
%  draw_body  --  full aerial-manipulator body (mirrors plot_result's
%       draw_full_manipulator, kept local so this script is self-contained).
% ========================================================================
function draw_body(ax, kin, params)
axis_len = 0.06;  arm_len = 0.650/2;  prop_rad = 15*0.0254/2;
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
r_c_I = r_0 + R_0 * kin.r_0c_0;

joint_color = [0.95 0.55 0.10];
link_color  = [0.30 0.30 0.30];
prop_color  = [0.30 0.55 0.95];
prop_face   = [0.65 0.80 0.98];

% quadrotor motors (X-config)
ang = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
motor_0 = zeros(3,4);  motor_I = zeros(3,4);
for m = 1:4
    motor_0(:,m) = arm_len * [cos(ang(m)); sin(ang(m)); 0];
    motor_I(:,m) = r_0 + R_0 * motor_0(:,m);
end
plot3(ax,[motor_I(1,1) motor_I(1,3)],[motor_I(2,1) motor_I(2,3)],[motor_I(3,1) motor_I(3,3)],'k-','LineWidth',2.0);
plot3(ax,[motor_I(1,2) motor_I(1,4)],[motor_I(2,2) motor_I(2,4)],[motor_I(3,2) motor_I(3,4)],'k-','LineWidth',2.0);
plot3(ax,motor_I(1,:),motor_I(2,:),motor_I(3,:),'ko','MarkerSize',9,'MarkerFaceColor',[0.15 0.15 0.15]);

% propeller disks
theta_c = linspace(0, 2*pi, 60);
for m = 1:4
    circ_local = [prop_rad*cos(theta_c); prop_rad*sin(theta_c); zeros(1,numel(theta_c))];
    circ_I     = r_0 + R_0 * (motor_0(:,m) + circ_local);
    patch('Parent',ax,'XData',circ_I(1,:),'YData',circ_I(2,:),'ZData',circ_I(3,:), ...
          'FaceColor',prop_face,'FaceAlpha',0.25,'EdgeColor',prop_color,'EdgeAlpha',0.85,'LineWidth',1.4);
end

% manipulator links (bent link 2 drawn as two segments) + joints
Plink = arm_link_polyline(O_I, R_I, params.l_i);
plot3(ax,Plink(1,:),Plink(2,:),Plink(3,:),'Color',link_color,'LineWidth',2.0);
for k = 1:n+1
    plot3(ax,O_I(1,k),O_I(2,k),O_I(3,k),'o','MarkerSize',9,'MarkerFaceColor',joint_color,'MarkerEdgeColor','k','LineWidth',1);
end

% end-effector + system CoM
plot3(ax,r_e_I(1),r_e_I(2),r_e_I(3),'p','MarkerSize',14,'MarkerFaceColor',[1.00 0.85 0.10],'MarkerEdgeColor','k','LineWidth',1.0);
plot3(ax,r_c_I(1),r_c_I(2),r_c_I(3),'^','MarkerSize',10,'MarkerFaceColor',[0.10 0.65 0.20],'MarkerEdgeColor','k','LineWidth',1.0);

% body frames (RGB triads)
draw_frame(ax, r_0, R_0, axis_len, 2.0);
for k = 2:n+1
    draw_frame(ax, O_I(:,k), R_I(:,:,k), axis_len, 2.0);
end
end

% ========================================================================
%  draw_frame  --  RGB axis triad (x=red, y=green, z=blue)
% ========================================================================
function draw_frame(ax, origin, R, L, lw)
ax_color = {'r','g','b'};
for a = 1:3
    quiver3(ax, origin(1),origin(2),origin(3), L*R(1,a),L*R(2,a),L*R(3,a), 0, ...
            'Color',ax_color{a},'LineWidth',lw,'MaxHeadSize',0.6);
end
end

% ========================================================================
%  draw_payload_cube  --  small solid cube centered at c, oriented by R.
% ========================================================================
function h = draw_payload_cube(ax, c, R, s, faceColor)
if nargin < 5, faceColor = [0.55 0.35 0.15]; end
V = 0.5*[ -1 -1 -1;  1 -1 -1;  1  1 -1; -1  1 -1; ...
          -1 -1  1;  1 -1  1;  1  1  1; -1  1  1 ];
V = (R * (s * V.')).' + c(:).';
F = [ 1 2 3 4;  5 6 7 8;  1 2 6 5;  2 3 7 6;  3 4 8 7;  4 1 5 8 ];
h = patch('Parent',ax,'Vertices',V,'Faces',F, ...
          'FaceColor',faceColor,'FaceAlpha',0.85,'EdgeColor','k','LineWidth',1.0);
end

% ========================================================================
%  payload_cube_size  --  cube edge (m); gently scales with payload mass.
% ========================================================================
function s = payload_cube_size(params)
m_p = 0.2;
if isfield(params,'payload_mass') && ~isempty(params.payload_mass)
    m_p = params.payload_mass;
end
s = 0.05 * (max(m_p,1e-3)/0.2)^(1/3);
end

% ========================================================================
%  ground_heading_arrow  --  drop a point to the floor (z = z_grd) and draw
%       the x-y projection of its body-x heading as an arrow from the foot.
%       Adds a faint vertical guide + a foot marker.  Returns the quiver
%       handle (empty if the heading is ~vertical, i.e. no x-y projection).
% ========================================================================
function h = ground_heading_arrow(ax, p_top, hdir, z_grd, L, col)
foot = [p_top(1); p_top(2); z_grd];
% faint vertical guide from the actual point down to its ground projection
plot3(ax, [p_top(1) foot(1)], [p_top(2) foot(2)], [p_top(3) foot(3)], ':', ...
      'Color',[0.6 0.6 0.6], 'LineWidth',0.8);
% foot marker at the ground projection
plot3(ax, foot(1), foot(2), foot(3), 'o', 'MarkerSize',5, ...
      'MarkerFaceColor',col, 'MarkerEdgeColor','k', 'LineWidth',0.5);
% horizontal projection of the heading direction
hxy = [hdir(1); hdir(2); 0];  nh = norm(hxy);
if nh < 1e-6, h = []; return; end          % heading ~ vertical: no x-y arrow
v = (L/nh) * hxy;
h = quiver3(ax, foot(1),foot(2),foot(3), v(1),v(2),v(3), 0, ...
            'Color',col, 'LineWidth',2.4, 'MaxHeadSize',0.8);
end