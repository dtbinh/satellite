% This function returns the conversion from dms to radian
% inputs         
%   deg         - angle in degree      [deg]
%   min         - angle in min         [min]
%   sec         - angle in sec         [sec]
%
% outputs       
%   rad         - angle in radian      [rad]
% 
% author        
%   rusty        - initial     05 jul 2018
%
% reference
%   
function rad = dms2rad(deg,min,sec)

  rad = (deg+min/60+sec/3600)*pi/180;

end
