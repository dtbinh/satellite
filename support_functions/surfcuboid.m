% function [R,A] = surfcuboid(dx,dy,dz)
% ------------------------------------------------------------------------
% This function outputs the position vectors and areas of each surface of a
% cuboid-shaped satellite.
% 
% ------------------------------------------------------------------------
function [R,A,N,dA] = surfcuboid(dx,dy,dz)
% Front Area
R(:,1)  = [dx/2;0;0];
A(:,1)  = [dz*dy;0;0];
N(:,1)  = R(:,1)/norm(R(:,1));
dA(:,1) = [dz;dy;0];

% Back Area
R(:,2) = [-dx/2;0;0];
A(:,2) = [-dz*dy;0;0];
N(:,2) = R(:,2)/norm(R(:,2));
dA(:,2) = [dz;dy;0];

% Right Area
R(:,3) = [0;dy/2;0];
A(:,3) = [0;dz*dx;0];
N(:,3) = R(:,3)/norm(R(:,3));
dA(:,3) = [dx;dz;0];

% Left Area
R(:,4) = [0;-dy/2;0];
A(:,4) = [0;-dz*dx;0];
N(:,4) = R(:,4)/norm(R(:,4));
dA(:,4) = [dx;dz;0];

% Bottom
R(:,5) = [0;0;dz/2];
A(:,5) = [0;0;dx*dy];
N(:,5) = R(:,5)/norm(R(:,5));
dA(:,5) = [dx;dy;0];

% Top Area
R(:,6) = [0;0;-dz/2];
A(:,6) = [0;0;-dx*dy];
N(:,6) = R(:,6)/norm(R(:,6));
dA(:,6) = [dx;dy;0];
        
end