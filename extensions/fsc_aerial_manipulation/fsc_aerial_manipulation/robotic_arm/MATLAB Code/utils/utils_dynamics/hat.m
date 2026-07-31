function v_hat = hat(v)
%HAT  Hat operator: 3-vector -> 3x3 skew-symmetric matrix.
%
%   v_hat = hat(v) returns the skew-symmetric matrix such that
%   hat(v)*u = cross(v,u) for any 3-vector u.
%
%   Implements eq. (1) of the reference paper:
%       hat(v) = [   0   -v3    v2 ;
%                   v3     0   -v1 ;
%                  -v2    v1     0 ]

    v_hat = [   0  , -v(3) ,  v(2) ;
              v(3) ,    0  , -v(1) ;
             -v(2) ,  v(1) ,    0  ];
end
