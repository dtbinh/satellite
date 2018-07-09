function R = Rquat(q)
 % q = [ eta eps1 eps2 eps3] Vector Rotation Propagation
if abs(norm(q)-1)>1e-6
    error('norm(q) must be equal to 1'); 
end

eta = q(1);     % Euler Angle Component of Quaternion
eps = q(2:4);   % Euler Vector Component of Quaternion

S = Smtrx(eps); % Skew Matrix of Euler Vector Component 

R = eye(3) + 2*eta*S + 2*S^2;