close all
clear all
clc
format long
% Parameters
global CONST

CONST.R   = 16;       % [Ohm] Terminal Resistance               1.43
CONST.L   = 26e-6;    % [H] Terminal Inductance, phase to phase 9.4e-3
CONST.eff = 0.51;     % [-] Efficiency
CONST.kt  = 0.902e-3; % [Nm/A] Torque Constant                  4/2
kt = CONST.kt;
CONST.ki  = 1.109e3;  % [A/Nm] Current constant
CONST.ke  = 9.4e-5/(2*pi/60) ;  % [V/(rad/s)] Back-EMF Constant 9.4e-5/(2*pi/60);
CONST.kn  = 10587*(2*pi/60);    % [(rad/s)/V] Speed Constant
CONST.J   = 0.125e-7+1.5465e-6;    % [kgm^2] Rotor Inertia                5.5e-3
J   = CONST.J;
CONST.Co  = 3.00e-6;  % [Nm] Friction Torque Static
CONST.Cv  = 0.52e-9/(2*pi/60);  % [Nm/(rad/s)] Friction Torque Dynamic 3.78e-03 
CONST.p   = 4;        % [-] Number of Poles
%% SPEED CONTROLLER
spd_kp = 0.03;
spd_ki = 0.002;
spd_kd = 0.0;

wtgt = 5000*2*pi/60;
Vtgt = 4;   

%% CURRENT CONTROLLER

i_kp = 10;
i_ki = 1e3;
i_kd = 2.0;

i_tgt = 0.01;

%% TORQUE CONTROLLER

trq_kp = 100;
trq_ki = 1e5;
trq_kd = 1e3;

trq_tgt = 0.05e-3; % [Nm] Maximum Torque allowed is 0.1 [mNm]

%% TIME 
dt     = 0.01;      % [sec] time step
tdur   = 5;          % [sec] time at final
tspan  = 0:dt:tdur;  % [sec] time array

sim('bldc_model',tdur)