% -------------------------------------------------------------------------
% BRUSHLESS DC MOTOR
% ------------------------------------------------------------------------
% This program runs the BLDC motor model in simulink and matlab. The BLDC
% model is modelled in detailed with the phases and commutation.
% ------------------------------------------------------------------------
close all
clear all
clc
format long
% ------------------------------------------------------------------------
% Parameters
global CONST

% BST DM24
R   = 2.5;                 % [Ohm] Terminal Resistance (2.1 Ohm)               
L   = 26e-6;               % [H] Terminal Inductance, phase to phase
eff = 0.77;                % [-] Efficiency               
ki  = 1.109e3;             % [A/Nm] Current constant (0.102A/mNm)
ke  = 1.026E-3/(2*pi/60) ; % [V/(rad/s)] Back-EMF Constant (1.026 mV/rpm)
kt  = 9.8e-3;                % [Nm/A] Torque Constant, also known as kM (9.8 mNm/A)  
kn  = 974*(2*pi/60);       % [(rad/s)/V] Speed Constant (974 rpm/V)
J   = 978.548e-6 + 6.5e-7; % [kgm2]                
Co  = 1e-3;                % [Nm] Friction Torque Static
Cv  = 3.5e-8/(2*pi/60);    % [Nm/(rad/s)] Friction Torque Dynamic
tau = 14e-3;               % [s] Mechanical Time Constant
p   = 6;                   % [-] Number of Poles
Vmax = 24;                 % [V] Max Voltage
Imax = 0.9;                % [A] Max current
Nmax = 5000*(2*pi/60);     % [rad/s]Maximum Speed
Tmax = 0.01;               % [Nm] Maximum Torque


CONST.R   = R;    % [Ohm] Terminal Resistance               
CONST.L   = L;    % [H] Terminal Inductance, phase to phase 
CONST.eff = eff;  % [-] Efficiency
CONST.kt  = kt;   % [Nm/A] Torque Constant                  
CONST.ki  = ki;   % [A/Nm] Current constant
CONST.ke  = ke;   % [V/(rad/s)] Back-EMF Constant 
CONST.kn  = kn;   % [(rad/s)/V] Speed Constant
CONST.J   = J;    % [kgm^2] Rotor Inertia                
CONST.Co  = Co ;  % [Nm] Friction Torque Static
CONST.Cv  = Cv ;  % [Nm/(rad/s)] Friction Torque Dynamic 
CONST.p   = p;    % [-] Number of Poles



%% CONTROL MODE

CTRL_MODE = 1; % CURRENT/TORQUE/SPEED/VOLTAGE

% ------------------------------------------------------
% CURRENT CONTROLLER
i_kp = 10; %1e2
i_ki = 0; %1e3;
i_kd = 0;

CONST.i_kp = i_kp;
CONST.i_ki = i_ki;
CONST.i_kd = i_kd;

i_tgt = 0.6;      % [A] Target current

%% INITIAL
theta_m_0 = 0; 
w_m_0     = 4000*rpm2rps;
wdot_dm_0 = 0;
% 5000 rpm - 83.33 rounds per sec - 0.012 sec/round
% 4000 rpm - 66.67 rounds per sec - 0.015 sec/round
% 3000 rpm - 50.00 rounds per sec - 0.020 sec/round
% 2000 rpm - 33.33 rounds per sec - 0.030 sec/round
% 1000 rpm - 16.67 rounds per sec - 0.060 sec/round
% 500  rpm -  8.33 rounds per sec - 0.120 sec/round
% 
%% TIME 
dt     = 0.00001;       % [sec] time step
tdur   = 1;           % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan);
thold  = 0.5;
CONST.dt = dt;
Vin = Vmax;

%% SIMULINK SOLVER
sim('bldc_model',tdur)

%% DISCRETE SIMULATION
theta_dm(1) = theta_m_0;
w_dm(1)     = w_m_0;
wdot_dm(1)  = wdot_dm_0; 
i_dm(1)     = 0; 

for i=1:1:length(tspan)-1

    switch CTRL_MODE
        case 1
            V_dm(i) = bldc_discrete_current(i_tgt,i_dm(i));
        case 2
            V_dm(i) = bldc_discrete_torque(trq_tgt(i),J*wdot_dm(i));  
        case 3
            V_dm(i) = bldc_discrete_speed(w_tgt,w_dm(i));
        case 4
            V_dm(i) = Vin;
    end
 
    if tspan(i)<thold
        V_dm(i) = 0; 

    end
    
    % Saturation for Voltage
    V_dm(i) = sat(V_dm(i),Vmax);
    
    outdm = bldc_discrete_model(V_dm(i),theta_dm(i),w_dm(i));

    theta_dm(i+1) = outdm(1);
    w_dm(i+1)     = outdm(2);
    wdot_dm(i+1)  = (w_dm(i+1)-w_dm(i))/dt;
    i_dm(i+1)     = 1/R*(V_dm(i)-1.17*ke*w_dm(i));
    V_dm(i+1)     = V_dm(i);
    
end

%% PLOT
close all

% figure
% plot(tspan,i_m(1,:),'b')
% grid on;hold on;
% if CTRL_MODE == 1
%     plot(tspan,i_tgt*ones(1,tlgth),'r--');
% end
% xlabel('Time [s]');
% ylabel('Current [A]');
% axis([-inf inf 0 1.0]);

figure
subplot(3,1,1)
plot(tspan,J*wdot_m(1,:)*1000,'b');
grid on;hold on;
plot(tspan,T_m1(1,:)*1000,'r');
xlabel('Time [s]');
ylabel('Torque [mNm]');
axis([-inf inf -1000 1000]);

subplot(3,1,2)
plot(tspan,w_m(1,:)*rps2rpm,'b');
grid on;hold on;
plot(tspan,w_m1(1,:)*rps2rpm,'r');
xlabel('Time [s]');
ylabel('Angular Velocity [rpm]');


subplot(3,1,3)
plot(tspan,V_dc_m(:,1),'b')
grid on;hold on;
xlabel('Time [s]');
ylabel('Voltage [V]');


