% function qout = qinvert(qin,type)
% --------------------------------------------------------------------------
% This function inverse the rotation of the quaternion input
% 
%              qout = qinvert(qin,'xyzw')
% 
% Input 
%     qin     -   quaternion
%     type    -   quaternion type
% 
% Output 
%     qout    -   quaternion inverted
% 
% Revision
%     rusty   -   initial     05 jul 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function qout = qinvert(qin,type)
qout = zeros(4,1);
switch type
    case 'wxyz' % input is wxyz
        qout(1,1) = qin(1,1);
        qout(2,1) = -qin(2,1);
        qout(3,1) = -qin(3,1);
        qout(4,1) = -qin(4,1);
        
    case 'xyzw' % input is xyzw
        qout(1,1) = -qin(1,1);
        qout(2,1) = -qin(2,1);
        qout(3,1) = -qin(3,1);
        qout(4,1) = qin(4,1);
        
    otherwise
  
end
end