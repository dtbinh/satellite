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

km  = 1/96.154;           % [Nm/A] Torque Constant, also known as kM (9.8 mNm/A)
ke  = 1.026E-3/(2*pi/60); % [V/(rad/s)]
J   = 978.548e-6;         % [kgm2] 
R   = 2.3;                % [R] 

i_tgt = 0.8;  % [A]
V_in  = 8.0;  % [V]

frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];

%% TIME 
dt     = 0.0001;    % [sec] time step resolution is 1 microsec
tdur   = 5;           % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan); % [n] step lengths

trq   = zeros(1,tlgth);  %[Nm]
spd   = zeros(1,tlgth);
ang   = zeros(1,tlgth);
spd_m = zeros(1,tlgth);
trq_m = zeros(1,tlgth);

t_cur = 0;
spd_m_old = 0;
tick_tot = 0;
spd(1) = 0*rpm2rps;  %[rad/s]

n_filter = 0.5;
interval = 1;

for i = 2:1:tlgth
    
    i_int   = -1/km*polyval(frct_p,spd(i-1));

    i_motor = i_tgt-i_int;
    
    trq(i) = km*i_motor;
    
    spd(i) = trq(i)/J*dt + spd(i-1);
    
    Vemf = ke*spd(i-1);
    Vmtr = R*i_motor;
    
    
    % Measurement
    ang(i) = spd(i)*dt + ang(i-1);
    if (ang(i)>=2*pi)
        ang(i) = 0;
        tick(i) = 1; 
    else
       tick(i) = 0;
    end
    
    tick_tot = tick_tot + tick(i);
    
    if (tick_tot>=interval)
        t_old = t_cur;
        t_cur = tspan(i);
        
        spd_m(i) = (1-n_filter)*interval*2*pi/(t_cur-t_old)+ n_filter*spd_m_old ;

        trq_m(i) = J*(spd_m(i)-spd_m_old)/(t_cur-t_old);
        spd_m_old = spd_m(i);
        
        tick_tot = 0;
    else
        spd_m(i) = spd_m(i-1);
        trq_m(i) = trq_m(i-1);
    end
    
    
    
end

%% PLOT

figure
plot(tspan,trq*1000);
grid on; hold on;
axis([-inf inf 2.5 3.5 ]);
xlabel('Time [sec');
ylabel('Torque [mNm]');


figure
plot(tspan,spd*rps2rpm);
grid on; hold on;
%axis([-inf inf 500 5000 ]);
xlabel('Time [sec');
ylabel('Speed [rpm]');
