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

if abs(in)< abs(max)
    out = in;
else
    out = max;
end

end