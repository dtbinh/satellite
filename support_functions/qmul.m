% function qout = qmul(q1,q2,type)
% -------------------------------------------------------------------------
% This program multiplies two quaternion sets.
%
% Inputs
%    q1 	- first quaternion set (xyzw)
%    q2     - second quaternion set (xyzw)
%
% Output
%    qout 	- q1.q2   case '.' - q1 is executed then q2
%    qout   - q1*q2   case '*' - q2 is executed then q1
% 
% Revision
%   rusty   - initiate    17 jul 2018
% 
% Reference
%   crassidis - Fundamentals Spacecraft Attitude Determination Control System
% 
% ------------------------------------------------------------------------
function qout = qmul(q1,q2,type)
if~exist('type','var')
   type = '*'; 
end

switch type

    case {'.'}
    qout(1,:) =   q1(4,:).*q2(1,:) - q1(3,:).*q2(2,:) + q1(2,:).*q2(3,:) + q1(1,:).*q2(4,:);    
    qout(2,:) =   q1(3,:).*q2(1,:) + q1(4,:).*q2(2,:) - q1(1,:).*q2(3,:) + q1(2,:).*q2(4,:);    
    qout(3,:) = - q1(2,:).*q2(1,:) + q1(1,:).*q2(2,:) + q1(4,:).*q2(3,:) + q1(3,:).*q2(4,:);    
    qout(4,:) = - q1(1,:).*q2(1,:) - q1(2,:).*q2(2,:) - q1(3,:).*q2(3,:) + q1(4,:).*q2(4,:);    

    case {'*','quat_mult','qmult'}
    
    qout(1,:) =   q1(4,:).*q2(1,:) + q1(3,:).*q2(2,:) - q1(2,:).*q2(3,:) + q1(1,:).*q2(4,:);
    qout(2,:) = - q1(3,:).*q2(1,:) + q1(4,:).*q2(2,:) + q1(1,:).*q2(3,:) + q1(2,:).*q2(4,:);
    qout(3,:) =   q1(2,:).*q2(1,:) - q1(1,:).*q2(2,:) + q1(4,:).*q2(3,:) + q1(3,:).*q2(4,:);
    qout(4,:) = - q1(1,:).*q2(1,:) - q1(2,:).*q2(2,:) - q1(3,:).*q2(3,:) + q1(4,:).*q2(4,:);
    
    otherwise
        
    fprintf('Quaternion Type is incorrect\n');    
        
end