% function v = vnorm(v)
% --------------------------------------------------------------------------
% This function inverse the rotation of the quaternion input
% Example
%              vout = vnorm([2;3;1])
% 
% Input 
%     qin     -   quaternion
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

function v = vnorm(v)

    v = v/sqrt(v'*v);          
 
return