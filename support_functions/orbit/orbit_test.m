clc
clear all

a     = (500+6378)*1000; % [m] Semi-major Axis 
incl  = 45*deg2rad;      % [rad] inclination
ecc   = 0;               % [ ] Eccentricity
omega = pi/3;               % [rad] Right Ascension of Node
nu    = 0;               % [rad] True Anomaly
argp  = 0;               % [rad] Argument of Perigee

fprintf('--------------------------------------------------Orbital Function Test------------------------------------------------------------------------------\n');
fprintf('Function |    SemiMajor        Incl       Ecc       RAsc        TA         AP   |            Position Vector          |           Velocity Vector     \n');
fprintf('-----------------------------------------------------------------------------------------------------------------------------------------------------\n');
% Curtis Model
[r,v]  = coe2rv(a, ecc,incl,omega,argp,nu,'curtis'); 
fprintf('coe2rv   | %14.5f %10.5f %10.5f %10.5f %10.5f %10.5f| %10.5f %10.5f %10.5f | %10.5f %10.5f %10.5f\n',a,incl,ecc,omega,argp,nu, r,v);

% Vallado Model
[r,v]  = coe2rv(a,ecc,incl,omega,argp,nu,'vallado');
fprintf('coe2rv   | %14.5f %10.5f %10.5f %10.5f %10.5f %10.5f| %10.5f %10.5f %10.5f | %10.5f %10.5f %10.5f\n',a,incl,ecc,omega,argp,nu, r,v);
% Vallado Model
[p,a,ecc,incl,omega,argp,nu] = rv2coe (r,v);
fprintf('rv2coe   | %14.5f %10.5f %10.5f %10.5f %10.5f %10.5f| %10.5f %10.5f %10.5f | %10.5f %10.5f %10.5f\n',a,incl,ecc,omega,argp,nu, r,v);

