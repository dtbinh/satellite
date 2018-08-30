close all
clear all
clc

createSimulation([0 0 1 1],2);
origin = [0;0;0];

% satellite
R_B_I = dcm(3,deg2rad(90))*dcm(1,deg2rad(90))

plotvector(R_B_I'*[1;0;0],origin,color('orange'),'x_{sat}')
plotvector(R_B_I'*[0;1;0],origin,color('orange'),'y_{sat}')
plotvector(R_B_I'*[0;0;1],origin,color('orange'),'z_{sat}')

% Inertial
u1 = vnorm([2;5;6]);
u2 = vnorm([-2;-1;7]);

v10 = dot(u1,u2)
vi1p0_1 = u1;
vi1p0_2 = u2 - v10 * u1            % Perpendicular to u1 in direction close to u2
vi1p0_2 = vnorm(vi1p0_2)

plotvector(u1,origin,'r','u1');
plotvector(u2,origin,'b','u2');

plotvector(vi1p0_2,origin,'r','vi1p0_2');

% Body Measured
b1 = R_B_I*u1;
b2 = R_B_I*u2;

v10b = dot(b1,b2)
vb1p0_1 = b1;
vb1p0_2 = b2 - v10b * b1            % Perpendicular to u1 in direction close to u2
vb1p0_2 = vnorm(vb1p0_2)

plotvector(R_B_I'*b1,origin,'r','b1',1.2,'--');
plotvector(R_B_I'*b2,origin,'r','b2',1.2,'--');

% Quest

