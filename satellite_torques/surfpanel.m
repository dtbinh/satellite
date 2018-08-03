% function [R,A] = surfpanel(dx,dy,dz)
% ------------------------------------------------------------------------
% This function outputs the position vectors and areas of each surface of a
% cuboid-shaped satellite.
% 
% ------------------------------------------------------------------------
function [R,A,N,dA] = surfpanel(dR,dx,dy)

% Solar Panel 1
R(:,1) = dR;
surf   = [1;0;0];
N(:,1) = surf/norm(surf);
dA(:,1)= [dx;dy;0];
A(:,1) = dA(1,1)*dA(2,1)*N(:,1);

R(:,2) = R;
surf = [-1;0;0];
N(:,2) = surf/norm(surf);
dA(:,2)= [dx;dy;0];
A(:,2) = dA(1,2)*dA(2,2)*N(:,2);