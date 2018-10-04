% function q = qnorm(q)
% --------------------------------------------------------------------------
% This function normalize the quaternion input
% Example
%              qout = qnorm(qin)
% 
% Input 
%     qin     -   quaternion
% 
% Output 
%     qout    -   quaternion normalized
% 
% Revision
%     rusty   -   initial     05 jul 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function q = qnorm(q)

qnorm = q'*q;

while (qnorm) > 1
    if qnorm > 1 + 1e-16
        q = ((3 + qnorm)/(1 + 3*qnorm))*q; % rescale quaternion to (err^3)/32     
    else
        q = q/sqrt(qnorm);                 % renormalize quaternion      
    end
        qnorm = q'*q;

end
return