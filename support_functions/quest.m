% function [R_qst,q_qst,J_qst] = quest(v1b,v2b, v1i, v2i)
% ---------------------------------------------------------------------------
% This function takes in two known vectors in inertial frame and two
% measured vectors in reference frame using Quest Method. The ouput is a Rotation Matrix.
% 
% Inputs
%	v1b - measure vector 1 in body frame
%   v2b - measure vector 2 in body frame
%   v1i - known vector 1 in inertial frame
%   v2i - known vector 2 in inertial frame
% 
% Output
%   R_qst - Rotation Matrix
%   q_qst - Quaternion
%
% Revision
%   rusty - 30 aug 2018
%
% Reference
%   chris hall http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/
%
% ----------------------------------------------------------------------

function [R_qst,q_qst,J_qst] = quest(v1b,v2b,v1i,v2i)
eig_opt = 1;
w1 = 1/2;
w2 = 1/2;

B = w1*(v1b*v1i') + w2*(v2b*v2i');
S = B + B';
Z = [B(2,3)-B(3,2);
     B(3,1)-B(1,3);
     B(1,2)-B(2,1)];
 
sigma = trace(B);

p = ((eig_opt+sigma)*eye(3) - S)^-1*Z;

qbar = 1/sqrt(1+p'*p)*[p;1];
eta = qbar(4);
eps = qbar(1:3);

R_qst = (eta^2 - eps'*eps)*eye(3) + 2*eps*eps'-2*eta*smtrx(eps);
q_qst = qbar;
J_qst = w1*(1 - v1b'*R_qst*v1i) + w2*(1 - v2b'*R_qst*v2i);


end

