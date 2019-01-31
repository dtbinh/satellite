% ----------------------------------------------------------------------
% 
% @brief This program runs the model rw_model for the BST RW-100 modelling.
%        User will need to define the control mode, current target, speed
%        target and torque target, inital speed and input voltage.
% 
% @author   Rusty Goh
% @date     25 January 2019
% 
% ----------------------------------------------------------------------

clear all
clc
format long

rpm2rps = 2*pi/60;

%% USER INPUT
% 1 - cur cmd               
% 2 - spd cmd               
% 3 - trq cmd (closed)
% 4 - trq cmd (open)
vol_in   = 8.0;           % [V] Input Voltage
spd_init = -5000*rpm2rps; %[rad/s] Initial Speed

ctrl_mode = 2;
cur_tgt = -0.6;          % [A]
spd_tgt = -2000*rpm2rps; % [rad/s]
trq_tgt = 0.006;         % [Nm]
model  = 'fm1';          % 'fm1',fm2',fm3'

% SIMULATION  PARAMETERS 
dt     = 0.001;         % [sec] time step 
tdur   = 60;            % [sec] time at final
time   = 0:dt:tdur;     % [sec] time array
tlgth  = length(time);  % [n] step lengths

% PlaceHolder
out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);

fprintf('Reaction Wheel Simulation\n');
 
for i = 1:1:tlgth
 [trq, spd, trq_m, spd_m, cur_cmd] = rw_model(model, ctrl_mode, cur_tgt, spd_tgt, trq_tgt, vol_in, spd_init, dt, time(i)) ;

% Output Read
out_trq_m(i)     = trq_m;
out_spd_m(i)     = spd_m;
out_trq(i)       = trq;
out_spd(i)       = spd;
out_cur_cmd(i)   = cur_cmd;

end

fprintf('Simulation ended\n');

%% PLOT
close all;
fprintf('Plotting Results\n');
screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;
screensetting = [0.25*screenwidth 0.25*screenheight screenwidth*0.5 screenheight*0.59];

% Speed
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_spd*rps2rpm);
grid on; hold on;
plot(time,out_spd_m*rps2rpm);
% axis([0 time(end) -5000 5000]);
xlabel('Time [sec]');
ylabel('Speed [rpm]');
title('Speed [rpm] vs Time [sec]');


% Torque
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_trq*1000);
grid on; hold on;
plot(time,out_trq_m*1000);
legend('trq','trq_{meas}');
% axis([-inf inf -30.0 30.0 ]);
xlabel('Time [sec]');
ylabel('Torque [mNm]');
title('Torque [mNm] vs Time [sec]');


% Current
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_cur_cmd*1000);
grid on; hold on;
legend('cur_c_m_d');
% axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec]');
ylabel('Current [mA]');
title('Current [mA] vs Time [sec]');


