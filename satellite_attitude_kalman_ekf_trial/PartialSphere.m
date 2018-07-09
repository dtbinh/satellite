function h_sp = PartialSphere(n, base_offset, radius, phi_max, face_color, edge_color, transparency)
% PARTIALSPHERE Generate partial sphere.

% CVS VERSION INFORMATION
% $Revision: 1.1 $
% $Author: HARDAC_S $
% $Date: 2005/03/14 14:14:18 $
% $Source: W:\\AOCS\\Aeolus_CVS_repository/OSE/code/support_functions/outputs/plot/PartialSphere.m,v $
% $Name:  $
% $State: Exp $


n2         = ceil(phi_max/2/pi*n);
r          = (0:1/n2:1)*sin(phi_max); 
theta      = (0:1/n:1)*2*pi;
factor     = radius/sin(phi_max);

[R, THETA] = meshgrid(r,theta);
X          = factor*R.*cos(THETA) + base_offset(1);
Y          = factor*R.*sin(THETA) + base_offset(2);
Z          = -sqrt(1-R.^2) + 1 + base_offset(3); 
h_sp       = mesh(X,Y,Z);

set(h_sp,'FaceColor',face_color)
set(h_sp,'FaceAlpha',transparency)
set(h_sp,'EdgeColor',edge_color)
set(h_sp,'EdgeAlpha',transparency)

