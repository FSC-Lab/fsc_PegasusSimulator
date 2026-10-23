function J0 = arm_task_jacobian(q, params)
%ARM_TASK_JACOBIAN  Arm-only end-effector task Jacobian J_3y^0 in {0}.
%
%   J0 = arm_task_jacobian(q, params) returns the 4 x n arm task Jacobian
%   (3 EE-position rows + 1 EE-yaw row) expressed in the base frame {0}.  This
%   is the matrix whose conditioning measures ARM (elbow/wrist) singularity --
%   the controller inverts it (u3 = J_3y \ inner).  It depends ONLY on the
%   joint angles q (invariant to the base pose r_0, R_0), so its singular
%   values / condition number are identical in {0} and {I}.
%
%   Mirrors the local J03y inside plan_compatible_trajectory.m, so the locked
%   and compatible cases report the SAME arm-conditioning measure.
%
%   INPUT  q      : n x 1 joint angles (rad)
%          params : needs .n, .h_i_im1 (3 x n axes), .l_i (3 x n offsets)

n = params.n;  h = params.h_i_im1;  li = params.l_i;
Ri = repmat(eye(3),1,1,n+1);
for i = 1:n, Ri(:,:,i+1) = Ri(:,:,i) * elem_rot(h(:,i), q(i)); end
h0 = zeros(3,n);   for i = 1:n, h0(:,i)  = Ri(:,:,i)   * h(:,i);  end   % h_k^0
O  = zeros(3,n+1); for i = 1:n, O(:,i+1) = O(:,i) + Ri(:,:,i+1)*li(:,i); end
r0e = O(:,n+1);    b3e = Ri(:,:,n+1)*[0;0;1];
Jp = zeros(3,n);   yaw = zeros(1,n);
for k = 1:n, Jp(:,k) = cross(h0(:,k), r0e - O(:,k));  yaw(k) = b3e.'*h0(:,k); end
J0 = [Jp; yaw];
end

function R = elem_rot(h, q)
H = [0 -h(3) h(2); h(3) 0 -h(1); -h(2) h(1) 0];
R = eye(3) + sin(q)*H + (1-cos(q))*(H*H);
end
