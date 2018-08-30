% function R = triad(v1b,v2b, v1i, v2i)
% -----------------------------------------------------------------------
% This function takes in two known vector in inertial frame and two
% measured vectors in reference frame. The ouput is a Rotation Matrix.
% 
% Inputs
%	v1b - measure vector 1 in body frame
%   v2b - measure vector 2 in body frame
%   v1i - known vector 1 in inertial frame
%   v2i - known vector 2 in inertial frame
% 
% Output
%   R_trd - Rotation Matrix
%   q_trd - Quaternion
%
% Revision
%   rusty - 30 aug 2018
%
% Reference
%   chris hall http://www.dept.aoe.vt.edu/~cdhall/courses/aoe4140/
%
% ----------------------------------------------------------------------

function [R_trd,q_trd,J_trd] = triad(v1b,v2b, v1i, v2i)
w1 = 1;
w2 = 1;

t1b = v1b;
t2b = cross(v1b, v2b)/norm(cross(v1b, v2b));
t3b = cross(t1b,t2b);

t1i = v1i;
t2i = cross(v1i,v2i)/norm(cross(v1i, v2i));
t3i = cross(t1i,t2i);

%% ESTIMATED ROTATION MATRIX
Rbt = [t1b t2b t3b];
Rti = [t1i t2i t3i]';

R_trd = Rbt*Rti;
q_trd = dcm2q(R_trd);
J_trd = w1*(1 - v1b'*R_trd*v1i) + w2*(1 - v2b'*R_trd*v2i);
end
