function [dyn, kin] = dynamics(X, params)
%DYNAMICS  Aerial-manipulator transformed + original dynamics (Flowchart PDF1).
%
%   [dyn, kin] = dynamics(X, params) computes the full set of transformed AND
%   transformed dynamic matrices for the aerial manipulator, following the
%   "Computation Flowchart -- Transformed Dynamics" (sections A-G):
%
%       A. Kinematics -- Rotation        (R_i^0, I_i^0, h_i^0)
%       B. Kinematics -- Translation     (positions, Jacobians A_i/A_e/A)
%       C. Kinematics -- Rotation (vel.) (h_dot, J_omega, omega, r_dot_O)
%       D. Simplified Notation           (A_dot, Xi_i, I_dot_i^0)
%       E. Transformed Dynamics -- Preliminary (M_rho, B, N1, L, W, ...)
%       F. Transformed Dynamics -- Whole-body  (M, C, g)
%       G. Transformed Dynamics -- End-effector (J_y, Lambda_y, J_y^#)
%
%   OUTPUTS
%       dyn : struct holding every quantity the controller (PDF2) consumes.
%       kin : struct of kinematic quantities for visualization.
%
%   Requires helper functions on the path:  joint_rotation(h,q), hat(v).

% ========================================================================
% 0. Unpack state and parameters
% ========================================================================
n       = params.n;            % number of manipulator joints
mi      = params.m_i;          % 1 x (n+1) masses; mi(1) = m_0 (quadrotor)
m_total = sum(mi);
I_i_i   = params.I_i_i;        % 3 x 3 x (n+1), inertia of body i in {i}
l_i     = params.l_i;          % 3 x n,  offset vector O_{i-1}->O_i in {i}
h_i_im1 = params.h_i_im1;      % 3 x n,  joint axis h_i^{i-1} (constant)
g_const = params.g;            % gravitational acceleration (m/s^2)

% State unpacking
r_0     = X(1:3);
R_0     = reshape(X(4:12), 3, 3);
q       = X(13:12+n);
v_0     = X(13+n:15+n);        %#ok<NASGU>  (kept for completeness)
omega_0 = X(16+n:18+n);
qdot    = X(19+n:18+2*n);

% ========================================================================
% A.  KINEMATICS -- Rotation
% ========================================================================
% ---- A.1  Per-joint rotations R_i^{i-1}(q_i)  -- eq. (7) ---------------
R_i_im1 = zeros(3,3,n);
for i = 1:n
    R_i_im1(:,:,i) = joint_rotation(h_i_im1(:,i), q(i)); % elementary rotation matrix
end

% ---- A.2  R_i^0  for i = 0..n  (slots k = 1..n+1)  -- eq. (7) ------------
R_i_0 = zeros(3,3,n+1);
R_i_0(:,:,1) = eye(3);                                % R_0^0 = I3
for i = 1:n
    R_i_0(:,:,i+1) = R_i_0(:,:,i) * R_i_im1(:,:,i);   % R_i^0 = R_{i-1}^0 R_i^{i-1}
end

% ---- A.3  R_e^0 = R_n^0  -- eq. (23) -------------------------------------
R_e_0 = R_i_0(:,:,n+1);

% ---- A.4  Inertia tensors in {0}  I_i^0 = R_i^0 * I_i^i * (R_i^0)' --- eq. (55) --
I_i_0 = zeros(3,3,n+1);
for i = 0:n
    R = R_i_0(:,:,i+1);                         % R_i^0
    I_i_0(:,:,i+1) = R * I_i_i(:,:,i+1) * R';   % R_0^i = (R_i^0)'
end

% ---- A.5  Joint axes h_i^0  for i = 0..n  -- eq. (15) -------------------
% h_0^0 = 0;  h_i^0 = R_{i-1}^0 * h_i^{i-1}  for i = 1..n
h_i_0 = zeros(3, n+1);
for i = 1:n
    h_i_0(:,i+1) = R_i_0(:,:,i) * h_i_im1(:,i);  % h_i^0 = R_{i-1}^0 * h_i^{i-1}
end

% ========================================================================
% B.  KINEMATICS -- Translation
% ========================================================================
% ---- B.1  Joint origins r_{0,O_i}^0  ------------------------------------
% r_{0,O_0}^0 = 0;  r_{0,O_i}^0 = r_{0,O_{i-1}}^0 + R_i^0 * l_i
r_0_Oi_0 = zeros(3, n+1);
for i = 1:n
    r_0_Oi_0(:,i+1) = r_0_Oi_0(:,i) + R_i_0(:,:,i+1) * l_i(:,i);
end

% ---- B.2  End-effector position r_{0e}^0 = r_{0,O_n}^0  -- eq. (22) -----
r_0e_0 = r_0_Oi_0(:,n+1);

% ---- B.3  CoM positions r_{0i}^0  -- eq. (6) -----------------------------
% r_{00}^0 = 0;  r_{0i}^0 = r_{0,O_{i-1}}^0 + R_i^0 * (l_i / 2)
r_0i_0 = zeros(3, n+1);
for i = 1:n
    r_0i_0(:,i+1) = r_0_Oi_0(:,i) + R_i_0(:,:,i+1) * (l_i(:,i) / 2);
end

% ---- B.4  System CoM r_{0c}^0  -- eq. (63) -------------------------------
% r_{0c}^0 = (1/m) * sum_{i=0}^{n} m_i * r_{0i}^0
r_0c_0 = zeros(3,1);
for i = 0:n
    r_0c_0 = r_0c_0 + mi(i+1) * r_0i_0(:,i+1);
end
r_0c_0 = r_0c_0 / m_total;

% ---- B.5  A_i = d r_{0i}^0 / dq  ----------------------------------------
% [A_i]_{:,k} = h_k^0 x (r_{0i}^0 - r_{0,O_{k-1}}^0),  k <= i
A_i = zeros(3, n, n+1);
for i = 1:n
    for k = 1:i
        A_i(:,k,i+1) = cross(h_i_0(:,k+1), r_0i_0(:,i+1) - r_0_Oi_0(:,k));
    end
end

% ---- B.6  A_e = d r_{0e}^0 / dq  ----------------------------------------
% [A_e]_{:,k} = h_k^0 x (r_{0e}^0 - r_{0,O_{k-1}}^0),  k = 1..n
A_e = zeros(3, n);
for k = 1:n
    A_e(:,k) = cross(h_i_0(:,k+1), r_0e_0 - r_0_Oi_0(:,k));
end

% ---- B.7  A = d r_{0c}^0 / dq = (1/m) * sum_{i=0}^{n} m_i * A_i --------
A = zeros(3, n);
for i = 0:n
    A = A + mi(i+1) * A_i(:,:,i+1);
end
A = A / m_total;

% ---- B.8  Tilde Jacobians:  A_tilde_i = A_i - A,  A_tilde_e = A_e - A --
A_tilde_i = zeros(3, n, n+1);
for i = 0:n
    A_tilde_i(:,:,i+1) = A_i(:,:,i+1) - A;
end
A_tilde_e = A_e - A;

% ---- B.9  Relative positions:  d_i^0 = r_{0i}^0 - r_{0c}^0 -------------
d_i_0 = zeros(3, n+1);
for i = 0:n
    d_i_0(:,i+1) = r_0i_0(:,i+1) - r_0c_0;
end
d_e_0 = r_0e_0 - r_0c_0;

% ========================================================================
% C.  KINEMATICS -- Rotation (velocity-level)
% ========================================================================
% ---- C.1  h_i^0 dot  ----------------------------------------------------
% h_dot_0^0 = 0;  h_dot_i^0 = sum_{j=1}^{i-1} hat(h_j^0) * h_i^0 * qdot_j
h_dot_i_0 = zeros(3, n+1);
for i = 2:n
    for j = 1:i-1
        h_dot_i_0(:,i+1) = h_dot_i_0(:,i+1) + cross(h_i_0(:,j+1), h_i_0(:,i+1)) * qdot(j);
    end
end

% ---- C.2  J_omega_i^q  for i = 0..n  -- eq. (16) ------------------------
% J_omega_0^q = 0;  J_omega_i^q = [h_1^0, ..., h_i^0, 0, ..., 0]
J_q_omega_i = zeros(3, n, n+1);
for i = 1:n
    for k = 1:i
        J_q_omega_i(:,k,i+1) = h_i_0(:,k+1);
    end
end

% ---- C.3  J_omega_e^q = J_omega_n^q  ------------------------------------
J_q_omega_e = J_q_omega_i(:,:,n+1);

% ---- C.4  J_omega_i^q dot  -- eq. (20) -----------------------------------
% [J_dot_omega_i^q]_{:,k} = h_dot_k^0  for k <= i
J_q_dot_omega_i = zeros(3, n, n+1);
for i = 1:n
    for k = 1:i
        J_q_dot_omega_i(:,k,i+1) = h_dot_i_0(:,k+1);
    end
end

% ---- C.5  J_omega_e^q dot = J_omega_n^q dot  ----------------------------
J_q_dot_omega_e = J_q_dot_omega_i(:,:,n+1);

% ---- C.6  omega_{0i}^0 = J_omega_i^q * qdot  -- eq. (14) ----------------
omega_0i_0 = zeros(3, n+1);
for i = 1:n
    omega_0i_0(:,i+1) = J_q_omega_i(:,:,i+1) * qdot;
end

% ---- C.7  omega_{0e}^0 = omega_{0n}^0  -----------------------------------
omega_0e_0 = omega_0i_0(:,n+1);

% ---- C.8  omega_i^0 = omega_0 + omega_{0i}^0  -- eq. (13) ---------------
omega_i_0 = zeros(3, n+1);
omega_i_0(:,1) = omega_0;
for i = 1:n
    omega_i_0(:,i+1) = omega_0 + omega_0i_0(:,i+1);
end

% ---- C.9  omega_e^0 = omega_n^0  -----------------------------------------
omega_e_0 = omega_i_0(:,n+1);

% ---- C.10  r_dot_{0,O_i}^0  ---------------------------------------------
% r_dot_{0,O_0}^0 = 0;  r_dot_{0,O_i}^0 = sum_{j=1}^{i} hat(omega_{0j}^0) R_j^0 l_j
r_dot_0_Oi_0 = zeros(3, n+1);
for i = 1:n
    for j = 1:i
        r_dot_0_Oi_0(:,i+1) = r_dot_0_Oi_0(:,i+1) + cross(omega_0i_0(:,j+1), R_i_0(:,:,j+1) * l_i(:,j));
    end
end

% ========================================================================
% D.  SIMPLIFIED NOTATION -- Time derivatives
% ========================================================================
% ---- D.1  A_i dot  -------------------------------------------------------
% [A_dot_i]_{:,k} = h_dot_k^0 x (r_{0i}^0 - r_{0,O_{k-1}}^0)
%                 + h_k^0     x (r_dot_{0i}^0 - r_dot_{0,O_{k-1}}^0)
r_dot_0i_0 = zeros(3, n+1);   % r_dot_{0i}^0 = A_i * qdot
for i = 1:n
    r_dot_0i_0(:,i+1) = A_i(:,:,i+1) * qdot;
end

A_dot_i = zeros(3, n, n+1);
for i = 1:n
    for k = 1:i
        A_dot_i(:,k,i+1) = cross(h_dot_i_0(:,k+1), r_0i_0(:,i+1)     - r_0_Oi_0(:,k)) ...
                         + cross(h_i_0(:,k+1),     r_dot_0i_0(:,i+1) - r_dot_0_Oi_0(:,k));
    end
end

% ---- D.2  A_e dot  -------------------------------------------------------
r_dot_0e_0 = A_e * qdot;      % r_dot_{0e}^0 = A_e * qdot
A_dot_e = zeros(3, n);
for k = 1:n
    A_dot_e(:,k) = cross(h_dot_i_0(:,k+1), r_0e_0     - r_0_Oi_0(:,k)) ...
                 + cross(h_i_0(:,k+1),     r_dot_0e_0 - r_dot_0_Oi_0(:,k));
end

% ---- D.3  A dot = (1/m) * sum_{i=0}^{n} m_i * A_dot_i ------------------
A_dot = zeros(3, n);
for i = 0:n
    A_dot = A_dot + mi(i+1) * A_dot_i(:,:,i+1);
end
A_dot = A_dot / m_total;

% ---- D.4  A_tilde dot:  A_tilde_dot_i = A_dot_i - A_dot -----------------
A_tilde_dot_i = zeros(3, n, n+1);
for i = 0:n
    A_tilde_dot_i(:,:,i+1) = A_dot_i(:,:,i+1) - A_dot;
end
A_tilde_dot_e = A_dot_e - A_dot;

% ---- D.5  d_i^0 dot = A_tilde_i * qdot,  d_e^0 dot = A_tilde_e * qdot --
d_dot_i_0 = zeros(3, n+1);
for i = 0:n
    d_dot_i_0(:,i+1) = A_tilde_i(:,:,i+1) * qdot;
end
d_dot_e_0 = A_tilde_e * qdot;

% ---- D.6  Xi_i = hat(omega_i^0) * I_i^0 + I_i^0 * hat(omega_0)  (3x3) --
Xi_i = zeros(3, 3, n+1);
for i = 0:n
    Xi_i(:,:,i+1) = hat(omega_i_0(:,i+1)) * I_i_0(:,:,i+1) ...
                  + I_i_0(:,:,i+1) * hat(omega_0);
end

% ---- D.7  I_i^0 dot = Xi_i + Xi_i'  -------------------------------------
I_dot_i_0 = zeros(3,3,n+1);
for i = 0:n
    I_dot_i_0(:,:,i+1) = Xi_i(:,:,i+1) + Xi_i(:,:,i+1)';
end

% ========================================================================
% E.  TRANSFORMED DYNAMICS -- Preliminary
% ========================================================================
% ---- E.1  M_rho = sum (m_i * A_tilde_i^T * A_tilde_i + J_omega_i^q^T * I_i^0 * J_omega_i^q) --
M_rho = zeros(n, n);
for i = 0:n
    M_rho = M_rho + mi(i+1) * A_tilde_i(:,:,i+1)' * A_tilde_i(:,:,i+1) ...
                  + J_q_omega_i(:,:,i+1)' * I_i_0(:,:,i+1) * J_q_omega_i(:,:,i+1);
end
% format long g
% disp(M_rho)

% ---- E.2  B = sum (-m_i * A_i^T * hat(d_i^0) + J_omega_i^q^T * I_i^0) --
B = zeros(n, 3);
for i = 0:n
    B = B - mi(i+1) * A_i(:,:,i+1)' * hat(d_i_0(:,i+1)) ...
          + J_q_omega_i(:,:,i+1)' * I_i_0(:,:,i+1);
end

% ---- E.3  N1 = M_rho^{-1} * B -------------------------------------------
N1 = M_rho \ B;

% ---- E.4  M_rho_dot ------------------------------------------------------
M_rho_dot = zeros(n, n);
for i = 0:n
    M_rho_dot = M_rho_dot ...
        + mi(i+1) * (A_tilde_dot_i(:,:,i+1)' * A_tilde_i(:,:,i+1) ...
                   + A_tilde_i(:,:,i+1)'     * A_tilde_dot_i(:,:,i+1)) ...
        + J_q_dot_omega_i(:,:,i+1)' * I_i_0(:,:,i+1)     * J_q_omega_i(:,:,i+1) ...
        + J_q_omega_i(:,:,i+1)'     * I_dot_i_0(:,:,i+1) * J_q_omega_i(:,:,i+1) ...
        + J_q_omega_i(:,:,i+1)'     * I_i_0(:,:,i+1)     * J_q_dot_omega_i(:,:,i+1);
end

% ---- E.5  B_dot ----------------------------------------------------------
B_dot = zeros(n, 3);
for i = 0:n
    B_dot = B_dot ...
        - mi(i+1) * (A_tilde_dot_i(:,:,i+1)' * hat(d_i_0(:,i+1)) ...
                   + A_tilde_i(:,:,i+1)'     * hat(d_dot_i_0(:,i+1))) ...
        + J_q_dot_omega_i(:,:,i+1)' * I_i_0(:,:,i+1) ...
        + J_q_omega_i(:,:,i+1)'     * I_dot_i_0(:,:,i+1);
end

% ---- E.6  N1_dot = M_rho^{-1} * (B_dot - M_rho_dot * N1) ---------------
N1_dot = M_rho \ (B_dot - M_rho_dot * N1);

% ---- E.7  L_i, L_e (each is 3x3)  ---------------------------------------
% L_i = -hat(d_i^0) - A_tilde_i * N1
L_i = zeros(3, 3, n+1);
for i = 0:n
    L_i(:,:,i+1) = -hat(d_i_0(:,i+1)) - A_tilde_i(:,:,i+1) * N1;
end
L_e = -hat(d_e_0) - A_tilde_e * N1;

% ---- E.8  L_i_dot, L_e_dot -----------------------------------------------
% L_dot_i = -(d_dot_i^0)^ - A_tilde_dot_i * N1 - A_tilde_i * N1_dot
L_dot_i = zeros(3, 3, n+1);
for i = 0:n
    L_dot_i(:,:,i+1) = -hat(d_dot_i_0(:,i+1)) ...
                       - A_tilde_dot_i(:,:,i+1) * N1 ...
                       - A_tilde_i(:,:,i+1)     * N1_dot;
end
L_dot_e = -hat(d_dot_e_0) - A_tilde_dot_e * N1 - A_tilde_e * N1_dot;

% ---- E.9  W_i, W_e -------------------------------------------------------
% W_i = I3 - J_omega_i^q * N1
W_i = zeros(3, 3, n+1);
for i = 0:n
    W_i(:,:,i+1) = eye(3) - J_q_omega_i(:,:,i+1) * N1;
end
W_e = eye(3) - J_q_omega_e * N1;

% ---- E.10  W_i_dot, W_e_dot ----------------------------------------------
% W_dot_i = -J_dot_omega_i^q * N1 - J_omega_i^q * N1_dot
W_dot_i = zeros(3, 3, n+1);
for i = 0:n
    W_dot_i(:,:,i+1) = -J_q_dot_omega_i(:,:,i+1) * N1 ...
                       - J_q_omega_i(:,:,i+1)    * N1_dot;
end
W_dot_e = -J_q_dot_omega_e * N1 - J_q_omega_e * N1_dot;

% ========================================================================
% F.  TRANSFORMED DYNAMICS -- Whole-body (tilde matrices)
% ========================================================================
% ---- F.1  M_tilde = blkdiag(M_t, M_r, M_rho) ----------------------------
M_t = m_total * eye(3);                                      % M_t = m * I3
M_r = zeros(3, 3);
for i = 0:n
    M_r = M_r + mi(i+1) * L_i(:,:,i+1)' * L_i(:,:,i+1) ...
              + W_i(:,:,i+1)' * I_i_0(:,:,i+1) * W_i(:,:,i+1);
end
M_tilde = blkdiag(M_t, M_r, M_rho);



% ---- F.2  g_tilde = [m*g*e3; 0; 0] --------------------------------------
g_tilde = [m_total * g_const * [0;0;1]; zeros(3,1); zeros(n,1)];

% ---- F.3  C_r ------------------------------------------------------------
C_r = zeros(3, 3);
for i = 0:n
    C_r = C_r + mi(i+1) * L_i(:,:,i+1)' * (hat(omega_0)*L_i(:,:,i+1) + L_dot_i(:,:,i+1)) ...
              + W_i(:,:,i+1)' * I_i_0(:,:,i+1) * W_dot_i(:,:,i+1) ...
              + W_i(:,:,i+1)' * Xi_i(:,:,i+1)  * W_i(:,:,i+1);
end

% ---- F.4  C_rp -----------------------------------------------------------
C_rp = zeros(3, n);
for i = 0:n
    C_rp = C_rp + mi(i+1) * L_i(:,:,i+1)' * (hat(omega_0)*A_tilde_i(:,:,i+1) + A_tilde_dot_i(:,:,i+1)) ...
               + W_i(:,:,i+1)' * I_i_0(:,:,i+1) * J_q_dot_omega_i(:,:,i+1) ...
               + W_i(:,:,i+1)' * Xi_i(:,:,i+1)  * J_q_omega_i(:,:,i+1);
end

% ---- F.5  C_p ------------------------------------------------------------
C_p = zeros(n, n);
for i = 0:n
    C_p = C_p + mi(i+1) * A_tilde_i(:,:,i+1)' * (hat(omega_0)*A_tilde_i(:,:,i+1) + A_tilde_dot_i(:,:,i+1)) ...
              + J_q_omega_i(:,:,i+1)' * I_i_0(:,:,i+1) * J_q_dot_omega_i(:,:,i+1) ...
              + J_q_omega_i(:,:,i+1)' * Xi_i(:,:,i+1)  * J_q_omega_i(:,:,i+1);
end

C_tilde = [zeros(3,3),  zeros(3,3),  zeros(3,n) ;
           zeros(3,3),  C_r,         C_rp       ;
           zeros(n,3), -C_rp',       C_p        ];

% ========================================================================
% G.  TRANSFORMED DYNAMICS -- End-effector
% ========================================================================
% R_0^e = (R_e^0)^T
R_0_e   = R_e_0';
e3T_R0e = R_0_e(3,:);   % e3^T * R_0^e   (1x3 row)

% ---- G.1  J_y ------------------------------------------------------------
% J_y = [ I3,        R0*L_e,        R0*A_tilde_e         ;
%         0_{1x3},   e3^T*R0^e*W_e, e3^T*R0^e*J_omega_e^q ]
J_y = [ eye(3),       R_0 * L_e,            R_0 * A_tilde_e        ;
        zeros(1,3),   e3T_R0e * W_e,        e3T_R0e * J_q_omega_e  ];

% ---- G.1b  Full end-effector geometric Jacobian J_e  (6 x (n+6)) ---------
% Maps the body velocity V = [v_0; omega_0; qdot] to the END-EFFECTOR body
% twist expressed in {e}:   J_e * V = [ v_e^e ; omega_e^e ]   (Image 2):
%   J_e = [ R_0^e, 0 ; 0, R_0^e ] [ I3, -hat(r_0e^0), A_e ; 0, I3, J_omega_e^q ]
% This is what couples an end-effector wrench w_e (force+moment, expressed in
% {e}) into the generalized dynamics through  M Vdot + C V + g = tau + J_e^T w_e.
J_e_inner = [ eye(3),      -hat(r_0e_0),  A_e          ;
              zeros(3,3),   eye(3),       J_q_omega_e  ];
J_e = blkdiag(R_0_e, R_0_e) * J_e_inner;

% ---- G.2  J_y_dot --------------------------------------------------------
top_row = R_0 * [ zeros(3,3), ...
                  hat(omega_0)*L_e       + L_dot_e, ...
                  hat(omega_0)*A_tilde_e + A_tilde_dot_e ];
bot_row = e3T_R0e * [ zeros(3,3), ...
                      W_dot_e         - hat(omega_0e_0)*W_e, ...
                      J_q_dot_omega_e - hat(omega_0e_0)*J_q_omega_e ];
J_y_dot = [ top_row ; bot_row ];

% ---- G.3  Lambda_y = (J_y * M_tilde^{-1} * J_y^T)^{-1} -----------------
Lambda_y = inv(J_y * (M_tilde \ J_y'));

% ---- G.4  J_y^# = M_tilde^{-1} * J_y^T * Lambda_y ----------------------
J_y_pinv = (M_tilde \ J_y') * Lambda_y;

% ---- G.5  (J_y^#)^T = [J_1y, J_2y, J_3y]  (column blocks) --------------
J_y_pinv_T = J_y_pinv';        % (J_y^#)^T
J_1y = J_y_pinv_T(:, 1:3);     % translational block
J_2y = J_y_pinv_T(:, 4:6);     % rotational block
J_3y = J_y_pinv_T(:, 7:end);   % joint block

% ========================================================================
% H.  Transformed-state map  T  (xi = T*V)
% ========================================================================
% T = [ R0, -R0*hat(r_0c^0), R0*A ; 0, I3, 0 ; 0, N1, In ]
T = [ R_0,         -R_0*hat(r_0c_0),  R_0*A          ;
      zeros(3,3),   eye(3),           zeros(3,n)     ;
      zeros(n,3),   N1,               eye(n)        ];

% ========================================================================
% I.  ORIGINAL (untransformed) DYNAMICS  -- eqs. (59),(60),(40)
% ========================================================================
% Per-body Jacobians (Flowchart "Original Dynamics"):
%   J_vi  = R0 [ I3,  -hat(r_0i^0),  A_i ]                       (3 x (n+6))
%   J_vi' =    [ I3,  -hat(r_0i^0),  A_i ]
%   Jdot_vi'= [ hat(om0), -(hat(om0)hat(r_0i^0)+hat(rdot_0i^0)),
%                                       (hat(om0)A_i + Adot_i) ]
%   J_wi  = [ 0_{3x3},  I3,  J_omega_i^q ]
%   Jdot_wi=[ 0_{3x3},  0_{3x3},  Jdot_omega_i^q ]
M = zeros(n+6, n+6);
C = zeros(n+6, n+6);
g = zeros(n+6, 1);
e3_ = [0;0;1];
for i = 0:n
    r0i    = r_0i_0(:,i+1);
    rdot0i = r_dot_0i_0(:,i+1);
    Ai     = A_i(:,:,i+1);
    Adi    = A_dot_i(:,:,i+1);
    Jw     = J_q_omega_i(:,:,i+1);
    Jwd    = J_q_dot_omega_i(:,:,i+1);
    Ii0    = I_i_0(:,:,i+1);

    % linear-velocity Jacobians
    Jvi_p     = [eye(3),       -hat(r0i),                                Ai];                 % J_vi'
    Jvi       = R_0 * Jvi_p;                                                                  % J_vi
    Jvi_p_dot = [hat(omega_0), -(hat(omega_0)*hat(r0i) + hat(rdot0i)),  (hat(omega_0)*Ai + Adi)]; % Jdot_vi'

    % angular-velocity Jacobians
    Jwi     = [zeros(3,3), eye(3),     Jw ];
    Jwi_dot = [zeros(3,3), zeros(3,3), Jwd];

    % accumulate  (eqs. 59, 60, 40)  -- reuse Xi_i = hat(wi^0)I_i^0 + I_i^0 hat(w0)
    M = M + mi(i+1) * (Jvi_p' * Jvi_p) + Jwi' * Ii0 * Jwi;
    C = C + mi(i+1) * (Jvi_p' * Jvi_p_dot) ...
          + Jwi' * Ii0 * Jwi_dot ...
          + Jwi' * Xi_i(:,:,i+1) * Jwi;
    g = g + Jvi' * (mi(i+1) * g_const * e3_);
end

% ========================================================================
% J.  Pack outputs for the controller (PDF2) and the simulator
% ========================================================================
% --- TRANSFORMED matrices (tilde) ---
dyn.M_tilde  = M_tilde;
dyn.M_t      = M_t;
dyn.M_r      = M_r;
dyn.M_rho    = M_rho;
dyn.C_tilde  = C_tilde;
dyn.C_r      = C_r;
dyn.C_rp     = C_rp;
dyn.C_p      = C_p;
dyn.g_tilde  = g_tilde;
% --- ORIGINAL (physical) matrices, for the real-dynamics integration ---
dyn.M        = M;
dyn.C        = C;
dyn.g        = g;
% --- Transformed-state map (u -> tau via tau = T'*[u1*R0*e3; u2; u3]) ---
dyn.T        = T;
% --- Coupling matrices (T blocks) ---
dyn.A        = A;
dyn.A_dot    = A_dot;
dyn.N1       = N1;
dyn.N1_dot   = N1_dot;
% --- End-effector task-space Jacobian and inertia ---
dyn.J_y      = J_y;
dyn.J_y_dot  = J_y_dot;
dyn.Lambda_y = Lambda_y;
dyn.J_y_pinv = J_y_pinv;
dyn.J_1y     = J_1y;
dyn.J_2y     = J_2y;
dyn.J_3y     = J_3y;
% --- Full end-effector geometric Jacobian (for the EE-wrench disturbance) ---
dyn.J_e      = J_e;          % 6 x (n+6),  J_e*V = [v_e^e; omega_e^e]
dyn.A_e      = A_e;          % 3 x n,  d r_{0e}^0 / dq
% --- Kinematic quantities needed to form references / errors ---
dyn.r_0c_0   = r_0c_0;       % system CoM in {0}
dyn.r_0e_0   = r_0e_0;       % end-effector in {0}
dyn.R_e_0    = R_e_0;        % R_e^0 (frame {e} relative to {0})
dyn.L_e      = L_e;
dyn.W_e      = W_e;
dyn.J_q_omega_e = J_q_omega_e;
dyn.J_q_dot_omega_e = J_q_dot_omega_e;   % d/dt J_{omega_e}^q  (3 x n) -- needed by controller omega_e_dot
dyn.omega_0e_0  = omega_0e_0;


% ========================================================================
% K.  Pack kinematic outputs (for visualization)
% ========================================================================
kin.r_0         = r_0;
kin.R_0         = R_0;
kin.q           = q;
kin.m_total     = m_total;
% Rotations
kin.R_i_im1     = R_i_im1;
kin.R_i_0       = R_i_0;
kin.R_e_0       = R_e_0;
kin.h_i_0       = h_i_0;
% Positions (in {0})
kin.r_0_Oi_0    = r_0_Oi_0;
kin.r_link_ends = r_0_Oi_0;   % alias for visualize_aerial_manipulator
kin.r_0i_0      = r_0i_0;
kin.r_0e_0      = r_0e_0;
kin.r_0c_0      = r_0c_0;
end