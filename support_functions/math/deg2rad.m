% function rad = deg2rad(deg)
% 
% This function returns the conversion from degree to radian
% 
%               rad = deg/180*pi;
% 
% inputs         
%   deg         - angle in degree         [deg]
%
% outputs       
%   rad         - angle in radian         [rad]
% 
% author        
%   rusty        - initial     05 jul 2018
%
% reference
%   
function rad = deg2rad(deg)

if ~exist('deg','var')
    rad = 1/180.0*pi;
else
    rad = deg/180.0*pi;
end

end

