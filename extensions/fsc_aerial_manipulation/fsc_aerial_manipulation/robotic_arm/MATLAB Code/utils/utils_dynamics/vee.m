function v = vee(M)
%VEE  Vee operator: inverse of hat().
%
%   v = vee(M) extracts the 3-vector v from a 3x3 skew-symmetric matrix M
%   such that hat(v) = M.

    v = [ M(3,2);
          M(1,3);
          M(2,1) ];
end
