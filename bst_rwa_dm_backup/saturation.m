% function out = sat(in,max)
% ------------------------------------------------------------------------
% This function saturates the input with a maximum saturation
%
% Input
%   in  -   Input to saturation
%   max -   Saturation Limit
% 
% Output
%   out -   Output saturated
%
% Revision
%   rusty   -   initate     24 jul 2018
%
% Reference
%   rusty
%-------------------------------------------------------------------------

function out = saturation(in,max)
if abs(in)< max
    out = in;
else
    fprintf('saturation\n');
    out = sign(in)*abs(max);
end

end