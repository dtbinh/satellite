% function [R_qst,q_qst,J_qst] = qmethod(v1b,v2b, v1i, v2i)
% ---------------------------------------------------------------------------
% This function takes in two known vectors in inertial frame and two
% measured vectors in reference frame using Q Method. The ouput is a Rotation Matrix.
% 
% Inputs
%	v1b - measure vector 1 in body frame
%   v2b - measure vector 2 in body frame
%   v1i - known vector 1 in inertial frame
%   v2i - known vector 2 in inertial frame
% 
% Output
%   R_qmd - Rotation Matrix
%   q_qmd - Quaternion
%
% Revision
%   rusty - 30 aug 2018
%
% Reference
%   chris hall http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/
%
% ----------------------------------------------------------------------
function [R_qmd,q_qmd,J_qmd] = qmethod(v1b,v2b, v1i, v2i)
w1 = 1;
w2 = 1;
B = w1*(v1b*v1i')+w2*(v2b*v2i');
S = B + B';
Z = [B(2,3)-B(3,2);B(3,1)-B(1,3);B(1,2)-B(2,1)];
sigma = trace(B);

K = [S-sigma*eye(3)   Z  ; 
          Z'        sigma];
      
[Keig_vec,Keig_val] = eig(K);
% Keigen_max = max(Keig_val);

q = Keig_vec(:,4);
eps = q(1:3);
eta = q(4);

R_qmd = (eta^2 - eps'*eps)*eye(3) + 2*eps*eps'-2*eta*smtrx(eps);
q_qmd = dcm2q(R_qmd);
J_qmd = w1*(1 - v1b'*R_qmd*v1i) + w2*(1 - v2b'*R_qmd*v2i);

end
