function att = ATT( quat )
%% Rotation Matrix of Quaternion
%  q = [eps1 eps2 eps3 eta] % Transformation

xi  = [ quat(4)*eye(3) + SKEW(quat(1:3,1)) ; -quat(1:3,1)' ]; % [John_Crassidis] Equation (A.174a)
psi = [ quat(4)*eye(3) - SKEW(quat(1:3,1)) ; -quat(1:3,1)' ]; % [John_Crassidis] Equation (A.174b)

att = xi' * psi;
return