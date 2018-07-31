% This function returns the conversion from rpm to rad/s
% inputs         
%   rpm        - ang. velocity in rpm        [rpm]
%
% outputs       
%   rps        - ang. velocity in rad/s      [rad/s]
% 
% author        
%   rusty        - initial     25 jul 2018
%
% reference
%   
function rps = rpm2rps(rpm)
if ~exist('rpm','var')
    rps = 2*pi/60;
else
    rps = rpm*2*pi/60;
end

end