% This function returns the conversion from rad/s to rpm
% inputs         
%   rps        - ang. velocity in rad/s    [rad/s]
%
% outputs       
%   rpm        - ang. velocity in rpm      [rpm]
% 
% author        
%   rusty        - initial     25 jul 2018
%
% reference
%   
function rpm = rps2rpm(rps)
if ~exist('rps','var')
    rpm = 60/2/pi;
else
    rpm = rps/2/pi*60;
end

end
