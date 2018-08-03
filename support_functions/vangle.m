% function v = vangle(v1,v2)
% --------------------------------------------------------------------------
% This function outputs the angle between two vectors
% 
% Input 
%     v1     -   vector 1
%     v2     -   vector 2
% 
% Output 
%     angle      -   angle between vectors
% 
% Revision
%     rusty   -   initial     05 jul 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function angle = vangle(v1,v2)

    angle = acos(dot(v1,v2)/norm(v1)/norm(v2));

return