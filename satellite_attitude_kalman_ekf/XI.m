%#eml
function xi = XI( quat )
%% Making Xi Matrix
xi = [ quat(4)*eye(3) + SKEW(quat(1:3,1)) ; 
            -quat(1:3,1)'                ];
return