% function deg = arcsec2deg(asec)
% ------------------------------------------------------------------------
% This function returns the conversion from radian to degree
%
% inputs       
%   asec        - angle in arcsec         ["]
% 
% outputs         
%   deg         - angle in deg         [deg]
%

% author        
%   rusty       - initial     05 aug 2018
%
% reference
%   

function deg = arcsec2deg(arcsec)

if ~exist('arcsec','var')
    deg = 1/3600*15;
else
    deg = arcsec/3600*15;
end

end