function  output = transform_vector(input)

q   = input(1:4); % instantaneous quaternion 
s_i = input(5:7);

% compute DCM
DCM = [q(1)^2-q(2)^2-q(3)^2+q(4)^2        2*(q(1)*q(2)+q(3)*q(4))        2*(q(1)*q(3)-q(2)*q(4));
           2*(q(1)*q(2)-q(3)*q(4))   -q(1)^2+q(2)^2-q(3)^2+q(4)^2	       2*(q(2)*q(3)+q(1)*q(4));
		   2*(q(1)*q(3)+q(2)*q(4))        2*(q(2)*q(3)-q(1)*q(4))   -q(1)^2-q(2)^2+q(3)^2+q(4)^2];
	   
% transform vector from one frame to another
output  = DCM*s_i;

return