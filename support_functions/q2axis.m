%  function [axis,angle] = q2axis(q,qtype)
%  -----------------------------------------------------------------------
%  This function outputs rotation axis and angle from input quaternions
% 
% Inputs
%   q       -   quaternion
%   qtype   -   quaternion type
% 
% Output
%   axis    -   [ - ] unit rotation axis
%   angle   -   [rad] rotation angle
% 
% Revision
%   rusty   -   intial      07 aug 2018
% 
% Reference
%   rusty
% 
% ------------------------------------------------------------------------

function [axis,angle] = q2axis(q,qtype)
if ~exist('qtype','var')
    qtype = 'xyzw';
end

switch qtype
    case 'xyzw'
        angle = 2*acos(q(4));
        axis = q(1:3)/norm(q(1:3));
    case 'wxyz'
        angle = 2*acos(q(1));   
        axis = q(2:4)/norm(q(2:4));
        
    otherwise
end