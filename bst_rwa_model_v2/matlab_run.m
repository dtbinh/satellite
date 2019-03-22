% ----------------------------------------------------------------------
% 
% @brief This program runs the model rw_model_v2 for the BST RW-100 modelling.
%        This version includes temperature modelling.
%        User will need to define the control mode, current target, speed
%        target and torque target, inital speed and input voltage.
% 
% @author   Rusty Goh
% @date     18 February 2019
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
vol_in   = 23.4;          % [V] Input Voltage
spd_init = 500*rpm2rps;  % [rad/s] Initial Speed

ctrl_mode = 3;
cur_tgt = 0.9;           % [A]
spd_tgt = -2000*rpm2rps; % [rad/s]
trq_tgt = 0.0065;        % [Nm]
model  = 'nexsat_fm3';          % 'fm1',fm2',fm3'
temp = 18;               % [degC]
test_spd_max = 5000*rpm2rps;

% SIMULATION  PARAMETERS 
dt     = 0.001;         % [sec] time step 
tdur   = 73.0;        % [sec] time at final
time   = 0:dt:tdur;     % [sec] time array
tlgth  = length(time);  % [n] step lengths

% PlaceHolder
out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);
out_trq_gross = zeros(1,tlgth);
out_km = zeros(1,tlgth);
out_km_exp = zeros(1,tlgth);
out_frct = zeros(1,tlgth);
out_cur_int = zeros(1,tlgth);

fprintf('Reaction Wheel Simulation\n');
 
for i = 1:1:tlgth

[trq, spd, trq_m, spd_m, cur_cmd,trq_gross,frct,km_exp,cur_int] = rw_model(model, ctrl_mode, cur_tgt, spd_tgt, trq_tgt, vol_in, spd_init, dt, time(i), temp) ;

% Output Read
out_trq_m(i)     = trq_m;
out_spd_m(i)     = spd_m;
out_trq(i)       = trq;
out_spd(i)       = spd;
out_cur_cmd(i)   = cur_cmd;
out_trq_gross(i) = trq_gross;
out_km(i) = trq_gross/cur_cmd;
out_km_exp(i) = km_exp;
out_frct(i) = frct;
out_cur_int(i) = cur_int;
% if (spd>test_spd_max)
%    break; 
% end
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
xlabel('Time [sec]');
ylabel('Speed [rpm]');
title('Speed [rpm] vs Time [sec]');
if (ctrl_mode == 1)&&(cur_tgt<0.0)
	axis([0 time(end) -5000 0]);
elseif (ctrl_mode == 1)&&(cur_tgt>0.0)
  axis([0 time(end) 0 5000]);
elseif (ctrl_mode ==2)&&(spd_tgt >0.0)
    axis([0 time(end) spd_init*rps2rpm 5000]);
elseif (ctrl_mode ==2)&&(spd_tgt <0.0)
	axis([0 time(end) -2000 -500]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==1500.0*rpm2rps)&&(trq_tgt==-0.002)
    axis([0 time(end) 400 1600]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==5000.0*rpm2rps)&&(trq_tgt==-0.002)
    axis([0 time(end) 3900 5100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==5000.0*rpm2rps)
    axis([0 time(end) 400 5100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==500.0*rpm2rps)&&(trq_tgt==0.0065)
    axis([0 time(end) 400 5100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==500.0*rpm2rps)
    axis([0 time(end) 400 1600]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==4000.0*rpm2rps)&&((trq_tgt==-0.001)||(trq_tgt==-0.006))
    axis([0 time(end) 3500 4100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==4000.0*rpm2rps)&&((trq_tgt==0.001)||(trq_tgt==0.006))
    axis([0 time(end) 3900 4500]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==4000.0*rpm2rps)
    axis([0 time(end) 3900 5100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==1000.0*rpm2rps)&&(trq_tgt==0.001)
    axis([0 time(end) 900 1500]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==1000.0*rpm2rps)&&(trq_tgt==-0.001)
    axis([0 time(end) 500 1100]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==1000.0*rpm2rps)&&(trq_tgt==0.006)
    axis([0 time(end) 900 1500]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init==1000.0*rpm2rps)&&(trq_tgt==-0.006)
    axis([0 time(end) 500 1100]);

end

% Torque
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_trq*1000);
grid on; hold on;
plot(time,out_trq_m*1000);
% plot(time,out_trq_gross*1000);
legend('trq','trq_{meas}');
xlabel('Time [sec]');
ylabel('Torque [mNm]');
title('Torque [mNm] vs Time [sec]');

if (ctrl_mode == 1)&&(cur_tgt>0.0)
    axis([0 time(end)  0.0 20.0 ]);
elseif (ctrl_mode == 1)&&(cur_tgt<0.0)
    axis([0 time(end) -20.0  0.0 ]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init*trq_tgt<0.0)
    axis([0 time(end)  -30.0 10.0 ]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init>=1000.0*rpm2rps)&&(trq_tgt==0.001)
    axis([0 time(end)  -1.0 4.0 ]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init>=1000.0*rpm2rps)&&(trq_tgt==-0.001)
    axis([0 time(end)  -30.0 10.0 ]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init>=1000.0*rpm2rps)&&(trq_tgt==0.006)
    axis([0 time(end) -1.0  9.0]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init*trq_tgt>0.0)&&(trq_tgt<0.005)
    axis([0 time(end)  -1.0 5.0 ]);
elseif ((ctrl_mode == 3)||(ctrl_mode == 4))&&(spd_init*trq_tgt>0.0)&&(trq_tgt>0.005)
    axis([0 time(end)  -1.0 10.0 ]);
end

% Current
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_cur_cmd*1000);
grid on; hold on;
legend('cur_c_m_d');
xlabel('Time [sec]');
ylabel('Current [mA]');
title('Current [mA] vs Time [sec]');
axis([0 time(end)  -1000.0 1000 ]);


if (ctrl_mode == 1)&&(cur_tgt~=0.0)

% KM
fig = figure;
set(fig,'Position',screensetting);
plot(out_spd_m*rps2rpm,out_km*1000);
grid on; hold on;
plot(out_spd_m*rps2rpm,out_km_exp*1000);
if (cur_tgt>0.0)
 axis([0 5000 8.0 14.0 ]);
else
 axis([-5000 0 8.0 14.0 ]);
end
title('KM [Nm/A] vs speed [rpm,]');

% Friction
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_frct*1000);
grid on; hold on;

title('Friction [mNm]vs Time [sec]');

% Internal Current
fig = figure;
set(fig,'Position',screensetting);
plot(time,out_cur_int*1000);
grid on; hold on;
legend('cur_c_m_d');
xlabel('Time [sec]');
ylabel('Internal Current [mA]'); 
title('Internal Current [mA] vs Time [sec]');

else
    fig = figure;
    set(fig,'Position',screensetting);
    plot(out_spd_m*rps2rpm,out_trq_m*1000);
    grid on; hold on;
    title('Torque [mNm] vs Speed[rpm]');
    axis([500 5000 -3.0 0.0 ]);
end