% function asec = rad2arcsec(rad)
%
% This function returns the conversion from radian to degree
%
%               asec = rad*  (180*3600)/pi;
%
% inputs         
%   rad         - angle in radian         [rad]
%
% outputs       
%   asec        - angle in arcsec         ["]
% 
% author        
%   rusty       - initial     05 aug 2018
%
% reference
%   

function asec = rad2arcsec(rad)
if ~exist('asec','var')
    asec = 1/pi*180*3600;
end

asec = rad/pi*180*3600;


end