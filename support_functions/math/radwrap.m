% function rad = radwrap(rad)
% ------------------------------------------------------------------------
% This function wraps angle between 0 and 2*pi
%
% inputs       
%   rad        - angle in rad         [rad]
% 
% outputs         
%   rad        - angle in rad         [rad]
%

% author        
%   rusty      - initial     10 aug 2018
%
% reference
%   rusty
%-------------------------------------------------------------------------

function rad = radwrap(rad)
    if ( rad < 0.0 )
        rad = rad + 2*pi;
    end
end