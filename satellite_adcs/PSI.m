%#eml
function psi = PSI( quat )
%% Making Psi Matrix
psi = [ quat(4)*eye(3) - SKEW(quat(1:3,1)) ; - quat(1:3,1)' ];
return