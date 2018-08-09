% function asec = deg2arcsec(deg)
% ------------------------------------------------------------------------
% This function returns the conversion from radian to degree
%
% inputs         
%   deg         - angle in deg         [deg]
%
% outputs       
%   asec        - angle in arcsec         ["]
% 
% author        
%   rusty       - initial     05 aug 2018
%
% reference
%   

function asec = deg2arcsec(deg)

if ~exist('deg','var')
    asec = 1/15*3600;
else
    asec = deg/15*3600;

end

end