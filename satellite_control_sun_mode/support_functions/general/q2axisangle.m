function [eigen_axis, eigen_angle] = q2axisangle(q)
% q2axisangle. Calculate eigenaxis & angle from quaternion

% Force correct quaternion orientation
q = reshape(q,4,1);

% Eigen vector - taking care to check for case where norm of 1st 3 elements is zero 
if norm(q(1:3))>1e-7
    eigen_axis = q(1:3)/norm(q(1:3));
else
    eigen_axis = [0 0 0]';
    q          = [0 0 0 1]';
end

% Eigen angle
eigen_angle = 2*acos(q(4));
if ~max(abs(eigen_axis*sin(eigen_angle) - q(1:3)))>1e-6
    eigen_angle = -eigen_angle;
end



