function R = joint_rotation(h, q)
%JOINT_ROTATION  Elementary rotation matrix for a revolute joint.
%
%   R = joint_rotation(h, q) returns R^{i-1}_i(q_i) as the standard
%   elementary rotation matrix about the x, y, or z axis:
%
%       Rx(q) = [ 1      0       0   ]
%               [ 0   cos(q)  -sin(q) ]
%               [ 0   sin(q)   cos(q) ]
%
%       Ry(q) = [ cos(q)   0   sin(q) ]
%               [   0      1     0    ]
%               [-sin(q)   0   cos(q) ]
%
%       Rz(q) = [ cos(q)  -sin(q)   0 ]
%               [ sin(q)   cos(q)   0 ]
%               [   0        0      1 ]
%
%   Inputs:
%       h : 3x1 unit joint axis in {i-1}, must be [1;0;0], [0;1;0], or [0;0;1].
%       q : scalar joint angle (rad).
%
%   Output:
%       R : 3x3 elementary rotation matrix in SO(3).

    cq = cos(q);  sq = sin(q);

    if     h(1) == 1        % x-axis
        R = [ 1,   0,    0  ;
              0,  cq,  -sq  ;
              0,  sq,   cq ];

    elseif h(2) == 1        % y-axis
        R = [ cq,  0,  sq ;
               0,  1,   0 ;
             -sq,  0,  cq ];

    elseif h(3) == 1        % z-axis
        R = [ cq,  -sq,  0 ;
              sq,   cq,  0 ;
               0,    0,  1 ];

    else
        error('joint_rotation: h must be a unit principal axis [1;0;0], [0;1;0], or [0;0;1].');
    end
end