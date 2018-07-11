function q3 = qmult(q1,q2)
% Calculate the product of two quaternions:
%    q3 = q1.q2

Q  = [ q2(4)  q2(3) -q2(2) q2(1); ...
      -q2(3)  q2(4)  q2(1) q2(2); ...  
       q2(2) -q2(1)  q2(4) q2(3); ...
      -q2(1) -q2(2) -q2(3) q2(4)];
q3 = Q*q1;
    