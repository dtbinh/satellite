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
R   = 2.5;                % [R] 
cur_max = 0.9;   % [A] Max current

V_in  = 8.0;  % [V]


frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];


%% CONTROLLER
% 1 - cur cmd
% 2 - spd cmd
% 3 - trq cmd (closed)
% 4 - trq cmd (open)

ctrl_mode = 4;

cur_tgt   = -0.8;         % [A]
spd_tgt = 5000*rpm2rps; % [rad/s]
trq_tgt = 0.0065;        % [Nm]

% TIME 
dt     = 0.001;         % [sec] time step resolution is 1 microsec
tdur   = 72.5;           % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [n] step lengths

trq   = zeros(1,tlgth);  % [Nm]
spd   = zeros(1,tlgth);
ang   = zeros(1,tlgth);
spd_m = zeros(1,tlgth);
trq_m = zeros(1,tlgth);
cur_cmd = zeros(1,tlgth);
% Inital Values
t_cur     = 0;
spd_m_old = 0;
tick_tot  = 0;
cur_motor = 0;
spd(1)    = 500*rpm2rps;  %[rad/s]

pid_trq_init = 1;

n_filter = 0.5;
interval = 1;

for i = 2:1:tlgth
    
    % Maximum allowable current
    cur_cmd(i) = saturation(cur_cmd(i-1),cur_max);
    
    % Back EMF Saturation
    iemf = (V_in-ke*spd(i-1))/R; %[V]
    cur_cmd(i) = saturation(cur_cmd(i),iemf);
    
    % Voltage check / Diode check
    Vemf = ke*spd(i-1); % [V] Voltage back emf
    Vmtr = R*cur_motor;   % [V] Voltage across motor
    
    if (abs((V_in*sign(cur_cmd)-Vemf-Vmtr))>abs(V_in*sign(cur_cmd)))
        fprintf('Internal Current\n');
        cur_int   = -1/km*polyval(frct_n,spd(i-1));
    else
        cur_int   = -1/km*polyval(frct_p,spd(i-1));
    end

    % Motor current
    cur_motor = cur_cmd(i)-cur_int;
    
    trq(i) = km*cur_motor;
    spd(i) = trq(i)/J*dt + spd(i-1);
    
    
    
    % Controller Step 
    switch ctrl_mode 
        
        case 1
            cur_cmd(i) = cur_tgt;
        case 2
            cur_cmd(i) = pid_controller(100E-3, 3.0e-05, 0.0, spd_tgt - spd(i-1),dt);
        case 3
            % Pre-load
            if (pid_trq_init)
              if(spd(i-1)*trq_tgt>0)
                    cur_cmd(i) = 1/km*(trq_tgt - polyval(frct_p,spd(i-1)));
              else
                  trq_f = polyval(frct_n,spd(i-1));
                  
                  if (abs(trq_tgt)>abs(trq_f))
                      cur_cmd(i) = 0;
                  else
                      cur_cmd(i) = 1/km*(trq_tgt - trq_f);
                  end
              end
              
            else
                
            end
            cur_cmd(i) = pid_controller(5.0, 0.03, 0.0, trq_tgt - trq(i-1),dt);
            
            pid_trq_init = 0;
            
            
            
        case 4
            if(spd(i-1)*trq_tgt>0)
                cur_cmd(i) = 1/km*(trq_tgt - polyval(frct_p,spd(i-1)));
            end
            
        otherwise
    end
    
    
    
    
    
    
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
close all;
figure
plot(tspan,trq*1000);
grid on; hold on;
axis([-inf inf -1.0 10.0 ]);
xlabel('Time [sec');
ylabel('Torque [mNm]');


figure
plot(tspan,spd*rps2rpm);
grid on; hold on;
%axis([-inf inf 500 5000 ]);
xlabel('Time [sec');
ylabel('Speed [rpm]');

figure
plot(tspan,cur_cmd*1000);
grid on; hold on;
axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec');
ylabel('Tgt_Cur [mA]');
