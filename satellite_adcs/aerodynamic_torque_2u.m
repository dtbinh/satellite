%% MODELLING OF AERODYNAMICS TORQUE 
close all
clear
clc

[R,A] = sat_geo('2u');

w  = [0;0;0.0511];      % [rad/s] Angular Velocity of Spacecraft in Body Frame
V  = [0;7.6686e3;0 ]; % [m/s] Velocity  of Spacecraft in Body Frame
rho = 4e-12    % [kg/m^3] density at Altitude 

torque = aero_torque(R,A,V,w,rho,'hughes')

torque = aero_torque(R,A,V,w,rho,'wertz')

