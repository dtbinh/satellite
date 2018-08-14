% function rad = arcsec2rad(asec)
%
% This function returns the conversion from radian to degree
%
%               rad = asec* pi / (180*3600);
%
% inputs         
%   asec        - angle in arcsec         ["]
% 
% outputs       
%   rad         - angle in radian         [rad]
% 
% author        
%   rusty       - initial     05 jul 2018
%
% reference
%   

function rad = arcsec2rad(asec)

if ~exist('asec','var')
    rad = pi/(180*3600);
else

    rad = asec * pi/(180*3600);
end

end