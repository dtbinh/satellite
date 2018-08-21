% This function returns the conversion from deg/s to rad/s
% inputs         
%   dps        - ang. velocity in deg/s      [deg/s]
% 
% outputs       
%   rps        - ang. velocity in rad/s    [rad/s]
% 
% author        
%   rusty        - initial     25 aug 2018
%
% reference
%   
function rps = dps2rps(dps)

if ~exist('dps','var')
    rps = pi/180;
else
    rps = dps/180*pi;
end

end
