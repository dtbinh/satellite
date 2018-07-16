% function qout = qconvert(qin,type)
% --------------------------------------------------------------------------
% This function converts the type of quaternion from either (xyzw) to (wxyz) 
% or (wxyz) to (xyzw) depending on the type input
% 
%              qout = qconvert(qin,'xyzw')
% 
% Input 
%     qin     -   quaternion
%     type    -   quaternion type
% 
% Output 
%     qout    -   quaternion converted
% 
% Revision
%     rusty   -   initial     05 jul 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function qout = qconvert(qin,type)
qout = zeros(4,1);
switch type
    case 'wxyz' % input is wxyz
        qout(1,1) = qin(2,1);
        qout(2,1) = qin(3,1);
        qout(3,1) = qin(4,1);
        qout(4,1) = qin(1,1);
        
    case 'xyzw' % input is xyzw
        qout(1,1) = qin(4,1);
        qout(2,1) = qin(1,1);
        qout(3,1) = qin(2,1);
        qout(4,1) = qin(3,1);
        
    otherwise
  
end
end