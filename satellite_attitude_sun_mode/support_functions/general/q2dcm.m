function  DCM = q2dcm(q)
% q2dcm. Calculates the direction cosine matrix corresponding to an
% attitude quaternion.
%
% Format:
%   DCM = q2dcm(q)

% compute DCM
DCM = [q(1)^2-q(2)^2-q(3)^2+q(4)^2        2*(q(1)*q(2)+q(3)*q(4))        2*(q(1)*q(3)-q(2)*q(4));
           2*(q(1)*q(2)-q(3)*q(4))   -q(1)^2+q(2)^2-q(3)^2+q(4)^2	       2*(q(2)*q(3)+q(1)*q(4));
		   2*(q(1)*q(3)+q(2)*q(4))        2*(q(2)*q(3)-q(1)*q(4))   -q(1)^2-q(2)^2+q(3)^2+q(4)^2];

return