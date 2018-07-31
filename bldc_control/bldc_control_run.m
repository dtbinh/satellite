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

%% VOLTAGE
Vmax = 4;

%% CONTROL MODE

CTRL_MODE = 3; % CURRENT/TORQUE/SPEED/VOLTAGE

% CURRENT CONTROLLER
i_kp = 100;
i_ki = 1e3;
i_kd = 0;

i_tgt = 0.15;     % [Nm] Maximum allowed is 0.15 [A]

% TORQUE CONTROL
trq_kp = 100;
trq_ki = 5e4;
trq_kd = 0;

trq_tgt = 0.05e-3; % [Nm] Maximum Torque allowed is 0.10 [mNm]

% SPEED CONTROL
spd_kp = 0.03;
spd_ki = 0.002;
spd_kd = 0.0;

CONST.spd_kp = spd_kp;
CONST.spd_ki = spd_ki;
CONST.spd_kd = spd_kd;

w_tgt = 5000*2*pi/60;

% VOLTAGE CONTROL
V_tgt = 4;   

%% INITIAL
theta_m_0 = 0;
w_m_0     = 0;

%% TIME 
dt     = 0.05;        % [sec] time step
tdur   = 30;          % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan);
CONST.dt = dt;

sim('bldc_model',tdur)

%% DISCRETE SIMULATION
Vin = V_tgt;

theta_dm(1) = theta_m_0;
w_dm(1)     = w_m_0;

for i=1:1:length(tspan)-1
    if tspan(i)<0.6
        V_dm(i) = 0;
    else
        
        V_dm(i) = bldc_speed_discrete(w_tgt,w_dm(i));
%         V_dm(i) = Vin;
    end
   
    V_dm(i) = sat(V_dm(i),Vmax);
    
    outdm = bldc_discrete(V_dm(i),theta_dm(i),w_dm(i));
    theta_dm(i+1) = outdm(1);
    w_dm(i+1)     = outdm(2);
    V_dm(i+1)     = V_dm(i);
end

%% PLOT
close all

figure
subplot(4,1,1)
plot(tspan,w_m(1,:)*rps2rpm)
grid on;hold on;
plot(tspan,w_sm(1,:)*rps2rpm)
plot(tspan,w_dm(1,:)*rps2rpm)
xlabel('Time [s]');
ylabel('Angular Velocity [rpm]');

subplot(4,1,2)
plot(tspan,J*wdot_m(1,:));
grid on;hold on;
plot(tspan,J*wdot_sm(1,:));
xlabel('Time [s]');
ylabel('Torque [Nm]');

subplot(4,1,3)
plot(tspan,V_dc_m)
grid on;hold on;
plot(tspan,V_dc_sm)
plot(tspan,V_dm)
xlabel('Time [s]');
ylabel('Voltage [V]');

subplot(4,1,4)
plot(tspan,i_m(1,:))
grid on;hold on;
plot(tspan,i_sm(1,:))
xlabel('Time [s]');
ylabel('Current [A]');

