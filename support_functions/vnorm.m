% function v = vnorm(v)
% --------------------------------------------------------------------------
% This function inverse the rotation of the vector input
% Example
%              vout = vnorm([2;3;1])
% 
% Input 
%     v    -   vector input
% 
% Output 
%     v    -   vector normalised
% 
% Revision
%     rusty   -   initial     05 jul 2018
% 
% Reference
%     rusty
%     
% -------------------------------------------------------------------------

function vout = vnorm(vin,eps_tol)
if ~exist('eps_tol','var')
    eps_tol = 1e-16;
end


if sqrt(vin'*vin) > eps_tol
    vout = vin/sqrt(vin'*vin);
else
    vout = 0;
end
 
return