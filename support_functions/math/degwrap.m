% function deg = degwrap(deg)
% ------------------------------------------------------------------------
% This function wraps angle between 0 and 360
%
% inputs       
%   deg        - angle in deg         [deg]
% 
% outputs         
%   deg        - angle in deg         [deg]
%

% author        
%   rusty      - initial     05 aug 2018
%
% reference
%   rusty
%-------------------------------------------------------------------------

function deg = degwrap(deg)
    if ( deg < 0.0 )
        deg = deg + 360;
    end
end