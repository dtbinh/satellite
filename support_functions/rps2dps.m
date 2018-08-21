% This function returns the conversion from rad/s to deg/s
% inputs         
%   rps        - ang. velocity in rad/s    [rad/s]
%
% outputs       
%   dps        - ang. velocity in deg/s      [deg/s]
% 
% author        
%   rusty        - initial     25 aug 2018
%
% reference
%   
function dps = rps2dps(rps)

if ~exist('rps','var')
    dps = 180/pi;
else
    dps = rps/pi*180;
end

end
