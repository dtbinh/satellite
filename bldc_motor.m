close all
clear all
clc
format long
% Parameters
global CONST

CONST.R   = 16;       % [Ohm] Terminal Resistance
CONST.L   = 26e-6;    % [H] Terminal Inductance, phase to phase
CONST.eff = 0.51;     % [-] Efficiency
CONST.kt  = 0.902e-3; % [Nm/A] Torque Constant
CONST.ki  = 1.109e3;  % [A/Nm] Current constant
CONST.ke  = 9.4e-5/(2*pi/60) ;  % [V/(rad/s)] Back-EMF Constant
CONST.kn  = 10587*(2*pi/60);    % [(rad/s)/V] Speed Constant
CONST.J   = 0.125e-7;    % [kgm^2] Rotor Inertia

CONST.Co  = 3.00e-6;  % [Nm] Friction Torque Static
CONST.Cv  = 0.52e-9/(2*pi/60);  % [Nm/(rad/s)] Friction Torque Dynamic

CONST.p   = 4;        % [-] Number of Poles

dt = 0.1;      % [sec] time step
tf = 10;        % [sec] time at final
t  = 0:dt:tf;   % [sec] time array
tl = length(t); % [sec] length of time array

% Pre-Allocate Space
xa    = zeros(7,tl);
x_0 = zeros(7,1);

options     = odeset('Maxstep',1.0,'OutputFcn',@outFcn);%,'RelTol',1e-12,'AbsTol',1e-12,'Refine',1);
[tout,yout] = ode45(@(t,x)bldc_dynamics(t,x),t,x_0);
