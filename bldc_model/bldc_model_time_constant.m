clear all
clc
format long

% 5000 rpm - 83.33 rounds per sec - 0.012 sec/round
% 4000 rpm - 66.67 rounds per sec - 0.015 sec/round
% 3000 rpm - 50.00 rounds per sec - 0.020 sec/round
% 2000 rpm - 33.33 rounds per sec - 0.030 sec/round
% 1000 rpm - 16.67 rounds per sec - 0.060 sec/round
% 500  rpm -  8.33 rounds per sec - 0.120 sec/round
% 

kt  = 9.8e-3;  % [Nm/A] Torque Constant, also known as kM (9.8 mNm/A) 
J   = 978.548e-6 + 6.5e-7; % [kgm2] 

i_tgt = 0.3;   % A

%% TIME 
dt     = 0.001;    % [sec] time step resolution is 1 microsec
tdur   = 5;           % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan); % [n] step lengths

trq   =  zeros(1,tlgth);  %[Nm]
trq_real  =  zeros(1,tlgth);  %[Nm]
spd   =  zeros(1,tlgth);
ang   = zeros(1,tlgth);
spd_m = zeros(1,tlgth);
trq_m = zeros(1,tlgth);
t_cur = 0;
spd_m_old = 0;
tick_tot = 0;
spd(1) = 4000*rpm2rps;  %[rad/s]

n_filter = 0.5;
interval = 2;
t_constant = 0.5;
for i = 2:1:tlgth
   t_cur = tspan(i);
   trq(i) = kt*i_tgt;
   trq_real(i) = kt*i_tgt*(1-exp(-t_cur/t_constant));
    
end

%% PLOT

figure
plot(tspan,trq*1000);
grid on; hold on;
plot(tspan,trq_real*1000);
%axis([-inf inf 2.5 3.5 ]);
xlabel('Time [sec');
ylabel('Torque [mNm]');
