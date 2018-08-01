close all
clear all
clc
format long
% Parameters
global CONST

R   = 16;                 % [Ohm] Terminal Resistance               
L   = 26e-6;              % [H] Terminal Inductance, phase to phase
eff = 0.51;               % [-] Efficiency
kt  = 0.902e-3;           % [Nm/A] Torque Constant                  
ki  = 1.109e3;            % [A/Nm] Current constant
ke  = 9.4e-5/(2*pi/60) ;  % [V/(rad/s)] Back-EMF Constant 
kn  = 10587*(2*pi/60);    % [(rad/s)/V] Speed Constant
J   = 0.125e-7+1.5465e-6;%0.125e-7; % [kgm^2] Rotor Inertia with wheel of 1.5465e-6 [kgm^2]               
Co  = 3.00e-6;            % [Nm] Friction Torque Static
Cv  = 0.52e-9/(2*pi/60);  % [Nm/(rad/s)] Friction Torque Dynamic  
p   = 4;                  % [-] Number of Poles

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

%% VOLTAGE
Vmax = 4;

%% CONTROL MODE

CTRL_MODE = 2; % CURRENT/TORQUE/SPEED/VOLTAGE

% CURRENT CONTROLLER
i_kp = 10; %1e2
i_ki = 1e3; %1e3;
i_kd = 0;

CONST.i_kp = i_kp;
CONST.i_ki = i_ki;
CONST.i_kd = i_kd;

i_tgt = 0.15;     % [Nm] Maximum allowed is 0.15 [A]

% TORQUE CONTROL
trq_kp = 1e3;
trq_ki = 5e4;
trq_kd = 0;

CONST.trq_kp = trq_kp;
CONST.trq_ki = trq_ki;
CONST.trq_kd = trq_kd;

trq_tgt = 0.02e-3; % [Nm] Maximum Torque allowed is 0.10 [mNm]

% SPEED CONTROL
spd_kp = 0.50;
spd_ki = 0.10;
spd_kd = 0.00;

CONST.spd_kp = spd_kp;
CONST.spd_ki = spd_ki;
CONST.spd_kd = spd_kd;

w_tgt = rpm2rps(2000);

% VOLTAGE CONTROL
V_tgt = 2.1;   

%% INITIAL
theta_m_0 = 0;
w_m_0     = 0;
wdot_dm_0 = 0;

%% TIME 
dt     = 0.01;        % [sec] time step
tdur   = 10;          % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan);
thold  = 0.5;
CONST.dt = dt;

sim('bldc_model',tdur)

%% DISCRETE SIMULATION
Vin = V_tgt;

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

figure

subplot(4,1,1)
plot(tspan,i_m(1,:))
grid on;hold on;
plot(tspan,i_sm(1,:))
plot(tspan,i_dm(1,:))
if CTRL_MODE == 1
    plot(tspan,i_tgt*ones(1,tlgth),'r--');
end
xlabel('Time [s]');
ylabel('Current [A]');

subplot(4,1,2)
plot(tspan,J*wdot_m(1,:));
grid on;hold on;
plot(tspan,J*wdot_sm(1,:));
plot(tspan,J*wdot_dm(1,:));
if CTRL_MODE == 2
    plot(tspan,trq_tgt*ones(1,tlgth),'r--');
end
xlabel('Time [s]');
ylabel('Torque [Nm]');

subplot(4,1,3)
plot(tspan,w_m(1,:)*rps2rpm)
grid on;hold on;
plot(tspan,w_sm(1,:)*rps2rpm)
plot(tspan,w_dm(1,:)*rps2rpm)
if CTRL_MODE == 3
    plot(tspan,w_tgt*rps2rpm*ones(1,tlgth),'r--');
end
xlabel('Time [s]');
ylabel('Angular Velocity [rpm]');


subplot(4,1,4)
plot(tspan,V_dc_m(1,:))
grid on;hold on;
plot(tspan,V_dc_sm)
plot(tspan,V_dm(1,:))
if CTRL_MODE == 4
    plot(tspan,V_tgt*ones(1,tlgth),'r--');
end
xlabel('Time [s]');
ylabel('Voltage [V]');


