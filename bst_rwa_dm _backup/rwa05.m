clear all
clc
format long
global rw_par rw_dat rw_ctrl pid_spd pid_trq open_trq
% 5000 rpm - 83.33 rounds per sec - 0.012 sec/round - 1 msec is 0.083 rds
% 4000 rpm - 66.67 rounds per sec - 0.015 sec/round - 1 msec is 0.067 rds
% 3000 rpm - 50.00 rounds per sec - 0.020 sec/round - 1 msec is 0.050 rds
% 2000 rpm - 33.33 rounds per sec - 0.030 sec/round
% 1000 rpm - 16.67 rounds per sec - 0.060 sec/round
% 500  rpm -  8.33 rounds per sec - 0.120 sec/round

rpm2rps = 2*pi/60;

%% USER INPUT
% 1 - cur cmd               <--- checked
% 2 - spd cmd               <--- checked
% 3 - trq cmd (closed)
% 4 - trq cmd (open)
vol_in   = 8.0;            % [V] Input Voltage
spd_init = -3000*rpm2rps;  %[rad/s] Initial Speed

ctrl_mode = 4;
cur_tgt = -0.6;          % [A]
spd_tgt = 3000*rpm2rps; % [rad/s]
trq_tgt = 0.006;       % [Nm]


%% SIMULATION  PARAMETERS 
dt     = 0.001;         % [sec] time step 
tdur   = 72;          % [sec] time at final
time    = 0:dt:tdur;     % [sec] time array
tlgth  = length(time); % [n] step lengths

out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);

 fprintf('Reaction Wheel Simulation\n');
 
for i = 1:1:tlgth

 [trq,spd,trq_m,spd_m,cur_cmd] = rw_model(ctrl_mode, cur_tgt, spd_tgt, trq_tgt, vol_in, spd_init, dt, time(i)) ;

% Output Read
out_trq_m(i)     = trq_m;
out_spd_m(i)     = spd_m;
out_trq(i)       = trq;
out_spd(i)       = spd;
out_cur_cmd(i)   = cur_cmd;

end


fprintf('ended\n');

%% PLOT
close all;
fprintf('getting plots\n');
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
axis([0 time(end) -5000 5000]);
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
axis([-inf inf -30.0 30.0 ]);
xlabel('Time [sec]');
ylabel('Torque [mNm]');
title('Torque [mNm] vs Time [sec]');


% Current
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_cur_cmd*1000);
grid on; hold on;
legend('cur_c_m_d');
axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec]');
ylabel('Current [mA]');
title('Current [mA] vs Time [sec]');
