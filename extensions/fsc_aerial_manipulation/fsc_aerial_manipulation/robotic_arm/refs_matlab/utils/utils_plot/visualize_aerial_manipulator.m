function visualize_aerial_manipulator(kin, params, varargin)
%VISUALIZE_AERIAL_MANIPULATOR  3-D visualization of the aerial-manipulator
%   configuration computed by closed_loop_dynamics.

% ---------------- Parse options ------------------------------------------
p = inputParser;
addParameter(p, 'AxisLength',    0.06);
addParameter(p, 'ArmLength',     0.650/2);        % 650 mm wheelbase -> 325 mm
addParameter(p, 'PropRadius',    15*0.0254/2);    % 1555 propeller: 15 inch
addParameter(p, 'Title',         'Aerial Manipulator Configuration');
addParameter(p, 'ShowCoM',       true);
addParameter(p, 'AxisLineWidth', 2.5);            % line width for all coordinate frame arrows
addParameter(p, 'DroneLineWidth',2.0);            % line width for quadrotor arms
addParameter(p, 'LinkLineWidth', 2.0);            % line width for manipulator links
parse(p, varargin{:});
axis_len  = p.Results.AxisLength;
arm_len   = p.Results.ArmLength;
prop_rad  = p.Results.PropRadius;
ttl       = p.Results.Title;
showCoM   = p.Results.ShowCoM;
axis_lw   = p.Results.AxisLineWidth;
drone_lw  = p.Results.DroneLineWidth;
link_lw   = p.Results.LinkLineWidth;

% ---------------- Pull configuration -------------------------------------
n           = params.n;
r_0         = kin.r_0;
R_0         = kin.R_0;
R_i_0       = kin.R_i_0;
r_link_ends = kin.r_link_ends;
r_0e_0      = kin.r_0e_0;
r_0c_0      = kin.r_0c_0;

% ---------------- Convert {0}-frame quantities to {I} --------------------
% Joint origins O_0..O_n in {I}  (O_0 = r_0, since r_link_ends(:,1) = 0)
O_I = zeros(3, n+1);
for k = 1:n+1
    O_I(:,k) = r_0 + R_0 * r_link_ends(:,k);
end

% Rotation of each link frame in {I}:  R_i^I = R_0 * R_i^0
R_I = zeros(3,3,n+1);
for k = 1:n+1
    R_I(:,:,k) = R_0 * R_i_0(:,:,k);
end

r_e_I = r_0 + R_0 * r_0e_0;   % end-effector in {I}
r_c_I = r_0 + R_0 * r_0c_0;   % system CoM in {I}

% ---------------- Figure setup -------------------------------------------
figure('Color','w', 'Name','Aerial Manipulator', 'Position', [80 80 980 800]);
hold on; grid on; box on; axis equal;
xlabel('$x_I$ (m)', 'Interpreter','latex'); ylabel('$y_I$ (m)', 'Interpreter','latex'); zlabel('$z_I$ (m)', 'Interpreter','latex');
view(35, 18);
title(ttl, 'FontWeight','bold', 'Interpreter','latex');
set(gca, 'FontSize', 10);

% ---------------- Inertial frame {I} at origin ---------------------------
draw_frame([0;0;0], eye(3), axis_len, axis_lw, 'I');

% ---------------- Quadrotor arms + propellers ----------------------------
joint_color = [0.95 0.55 0.10];   % orange  -- used for ALL joint dots

% Motor positions in body frame {0}  (X-config: 45° offsets)
ang = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4];
motor_0 = zeros(3,4);
for m = 1:4
    motor_0(:,m) = arm_len * [cos(ang(m)); sin(ang(m)); 0];
end
motor_I = zeros(3,4);
for m = 1:4
    motor_I(:,m) = r_0 + R_0 * motor_0(:,m);
end

% Two crossing arms
plot3([motor_I(1,1) motor_I(1,3)], [motor_I(2,1) motor_I(2,3)], ...
      [motor_I(3,1) motor_I(3,3)], 'k-', 'LineWidth', drone_lw);
plot3([motor_I(1,2) motor_I(1,4)], [motor_I(2,2) motor_I(2,4)], ...
      [motor_I(3,2) motor_I(3,4)], 'k-', 'LineWidth', drone_lw);

% Four motor dots (black)
plot3(motor_I(1,:), motor_I(2,:), motor_I(3,:), ...
      'ko', 'MarkerSize', 9, 'MarkerFaceColor', [0.15 0.15 0.15]);

% Four propeller disks
theta_c    = linspace(0, 2*pi, 60);
prop_color = [0.30 0.55 0.95];
prop_face  = [0.65 0.80 0.98];
for m = 1:4
    circ_local = [prop_rad*cos(theta_c); prop_rad*sin(theta_c); zeros(1,numel(theta_c))];
    circ_I     = r_0 + R_0 * (motor_0(:,m) + circ_local);
    patch(circ_I(1,:), circ_I(2,:), circ_I(3,:), prop_face, ...
          'FaceAlpha',0.25, 'EdgeColor',prop_color, 'EdgeAlpha',0.85, 'LineWidth',1.4);
end

% ---------------- Joint dots  O_0 .. O_n  (all orange) ------------------
% O_0 is the quadrotor centre = arm base
for k = 1:n+1
    plot3(O_I(1,k), O_I(2,k), O_I(3,k), 'o', 'MarkerSize', 9, ...
          'MarkerFaceColor', joint_color, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
end

% ---------------- Manipulator links (bent link 2 = two segments) --------
link_color = [0.30 0.30 0.30];
Plink = arm_link_polyline(O_I, R_I, params.l_i);
plot3(Plink(1,:), Plink(2,:), Plink(3,:), 'Color', link_color, 'LineWidth', link_lw);

% End-effector marker
plot3(r_e_I(1), r_e_I(2), r_e_I(3), 'p', 'MarkerSize', 14, ...
      'MarkerFaceColor', [1.00 0.85 0.10], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

% ---------------- Body frames -------------------------------------------
% Axes only (label=''); then add {k} text right beside each orange joint dot.
% Small fixed offset: slightly right (+x_I) and up (+z_I) — always close to dot.
lbl_off = 0.3*[axis_len; axis_len; axis_len];   % tune here if needed
draw_frame(r_0, R_0, axis_len, axis_lw, '');
text(r_0(1)+lbl_off(1), r_0(2)+lbl_off(2), r_0(3)+lbl_off(3), ...
     '$\{0\}$', 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05], ...
     'Interpreter', 'latex');
for k = 2:n+1
    draw_frame(O_I(:,k), R_I(:,:,k), axis_len, axis_lw, '');
    if k == n+1
        lbl = ['$\{' num2str(k-1) '/e\}$'];
    else
        lbl = ['$\{' num2str(k-1) '\}$'];
    end
    text(O_I(1,k)+lbl_off(1), O_I(2,k)+lbl_off(2), O_I(3,k)+lbl_off(3), ...
         lbl, 'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05], ...
         'Interpreter', 'latex');
end

% ---------------- Optional system CoM -----------------------------------
h_com = [];
if showCoM
    h_com = plot3(r_c_I(1), r_c_I(2), r_c_I(3), '^', ...
                  'MarkerSize', 10, 'MarkerFaceColor', [0.10 0.65 0.20], ...
                  'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
end

% ---------------- Axis limits -------------------------------------------
all_pts = [O_I, motor_I, r_e_I, r_c_I, [0;0;0]];
pad = max(0.25, 0.5*axis_len + 0.15);
xlim([min(all_pts(1,:))-pad, max(all_pts(1,:))+pad]);
ylim([min(all_pts(2,:))-pad, max(all_pts(2,:))+pad]);
zlim([min(all_pts(3,:))-pad, max(all_pts(3,:))+pad]);

% ---------------- Legend ------------------------------------------------
h_dx = plot3(nan,nan,nan,'r-','LineWidth',axis_lw);
h_dy = plot3(nan,nan,nan,'g-','LineWidth',axis_lw);
h_dz = plot3(nan,nan,nan,'b-','LineWidth',axis_lw);
h_qa = plot3(nan,nan,nan,'k-','LineWidth',drone_lw);
h_pr = plot3(nan,nan,nan,'-','Color',prop_color,'LineWidth',1.4);
h_ml = plot3(nan,nan,nan,'-','Color',link_color,'LineWidth',link_lw);
h_jt = plot3(nan,nan,nan,'o','MarkerSize',8, ...
             'MarkerFaceColor',joint_color,'MarkerEdgeColor','k');
h_ee = plot3(nan,nan,nan,'p','MarkerSize',12, ...
             'MarkerFaceColor',[1.00 0.85 0.10],'MarkerEdgeColor','k');
leg_h = [h_dx,h_dy,h_dz,h_qa,h_pr,h_ml,h_jt,h_ee];
leg_l = {'$x$-axis','$y$-axis','$z$-axis','Quadrotor arms', ...
         'Propeller disks','Manipulator links','Joints $O_i$','End-effector'};
if showCoM
    leg_h = [leg_h, h_com];
    leg_l = [leg_l, {'System CoM $r_c$'}];
end
legend(leg_h, leg_l, 'Location','eastoutside', 'FontSize',9, 'Interpreter','latex');

hold off;
end

% ========================================================================
%  draw_frame: RGB triad with label placed along local x-axis tip
% ========================================================================
function draw_frame(origin, R, L, lw, label)
%DRAW_FRAME  Fixed-length RGB axis arrows (x=red,y=green,z=blue).
%   Label is placed slightly past the x-axis tip so each frame's label
%   naturally spreads in the direction that frame's x-axis points,
%   avoiding the vertical stacking that occurs with a global z_I offset.
ax_color = {'r','g','b'};
for ax = 1:3
    quiver3(origin(1), origin(2), origin(3), ...
            L*R(1,ax), L*R(2,ax), L*R(3,ax), 0, ...
            'Color', ax_color{ax}, 'LineWidth', lw, 'MaxHeadSize', 0.6);
end
if nargin >= 5 && ~isempty(label)
    % Offset label along the local x-axis tip + tiny z_I lift
    lbl = origin + 1.15*L*R(:,1) + [0;0;0.4*L];
    text(lbl(1), lbl(2), lbl(3), ['$\{' label '\}$'], ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.05 0.05 0.05], 'Interpreter', 'latex');
end
end