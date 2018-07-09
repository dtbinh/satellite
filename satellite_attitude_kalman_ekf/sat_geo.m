function [R,A] = sat_geo(type)

switch type
    case '1u'
              
        dx = 0.1; % [m] length of x
        dy = 0.1; % [m] length of y
        dz = 0.1; % [m] length of z
        
    case '2u'
        
        dx = 0.1; % [m] length of x
        dy = 0.1; % [m] length of y
        dz = 0.2; % [m] length of z
        
    case '3u'
              
        dx = 0.1; % [m] length of x
        dy = 0.1; % [m] length of y
        dz = 0.3; % [m] length of z
        
    otherwise
end

% Front Area
R(:,1) = [dx/2;0;0];
A(:,1) = [dz*dy;0;0];
        
% Back Area
R(:,2) = [-dx/2;0;0];
A(:,2) = [-dz*dy;0;0];
        
% Right Area
R(:,3) = [0;dy/2;0];
A(:,3) = [0;dz*dx;0];

% Left Area
R(:,4) = [0;-dy/2;0];
A(:,4) = [0;-dz*dx;0];

% Bottom
R(:,5) = [0;0;dz/2];
A(:,5) = [0;0;dx*dy];
           
% Top Area
R(:,6) = [0;0;-dz/2];
A(:,6) = [0;0;-dx*dy];

% Solar Sail Area
% R(:,7) = [0;0;0.5];        
% A(:,7) = [0;-1;0];
        
        
end