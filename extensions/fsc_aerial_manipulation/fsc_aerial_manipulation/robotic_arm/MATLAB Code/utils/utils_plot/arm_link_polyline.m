function P = arm_link_polyline(O_I, R_I, l_i)
%ARM_LINK_POLYLINE  Ordered points for drawing the manipulator links, with any
%   BENT link rendered as TWO straight segments instead of a direct line.
%
%   P = arm_link_polyline(O_I, R_I, l_i) returns a 3 x M list of points to
%   connect with a single plot3 call.  A link whose offset l_i(:,k) has BOTH an
%   axial (z) and a lateral (x/y) component -- e.g. the bent link 2,
%   l_2 = delta2*e1 - l2*e3 -- gets an extra CORNER vertex so it is drawn as
%   O_{k-1} -> corner -> O_k.  The corner drops the axial (z) part first, then
%   steps laterally to O_k, matching singularity_sweep.m/armpoly:
%       Ob = O_{k-1} + R_k * [0; 0; l_i(3,k)] .
%   Straight (single-axis) links add no corner (the corner would coincide with
%   an endpoint), so links 1, 3, 4 render as a single segment as before.  No
%   marker is placed at the corner (joints are drawn separately at O_I).
%
%   INPUT  O_I : 3 x (n+1) joint origins O_0..O_n in the drawing frame
%          R_I : 3 x 3 x (n+1) frame rotations; R_I(:,:,k+1) is frame {k}
%          l_i : 3 x n link offset vectors (O_{k-1} -> O_k, expressed in {k})

n = size(l_i, 2);
P = O_I(:,1);                                   % start at O_0
for k = 1:n
    lk   = l_i(:,k);
    bent = abs(lk(3)) > 1e-9 && hypot(lk(1), lk(2)) > 1e-9;
    if bent
        Ob = O_I(:,k) + R_I(:,:,k+1) * [0; 0; lk(3)];   % corner: drop z first
        P  = [P, Ob];                                   %#ok<AGROW>
    end
    P = [P, O_I(:,k+1)];                                 %#ok<AGROW>
end
end
