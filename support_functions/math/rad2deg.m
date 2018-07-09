% function deg = rad2deg(rad)
%
% This function returns the conversion from radian to degree
%
%               deg = rad*180/pi;
%
% inputs         
%   rad         - angle in radian         [rad]
%
% outputs       
%   deg         - angle in degree         [deg]
% 
% author        
%   rusty       - initial     05 jul 2018
%
% reference
%   
function deg = rad2deg(rad)

if ~exist('rad','var')
    deg = 1*180/pi;
else
    deg = rad*180/pi;
end


end

