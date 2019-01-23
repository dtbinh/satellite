clear all
clc
format long

% 5000 rpm - 83.33 rounds per sec - 0.012 sec/round - 1 msec is 0.083 rds
% 4000 rpm - 66.67 rounds per sec - 0.015 sec/round - 1 msec is 0.067 rds
% 3000 rpm - 50.00 rounds per sec - 0.020 sec/round - 1 msec is 0.050 rds
% 2000 rpm - 33.33 rounds per sec - 0.030 sec/round
% 1000 rpm - 16.67 rounds per sec - 0.060 sec/round
% 500  rpm -  8.33 rounds per sec - 0.120 sec/round
% 
ki  = 96.154;             % [A/Nm] Current Constant
km  = 1/ki;               % [Nm/A] Torque Constant
ke  = 1.026E-3/(2*pi/60); % [V/(rad/s)] Back-emf constant
J   = 978.548e-6;         % [kgm2] Moment of Inertia
R   = 2.5;                % [R] Terminal Resistance
cur_max = 0.9;            % [A] Max Allowable current

V_in  = 8.0;              % [V] Input Voltage

% Friction Model
frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];


%% CONTROLLER
% 1 - cur cmd               <--- checked
% 2 - spd cmd               <--- checked
% 3 - trq cmd (closed)
% 4 - trq cmd (open)

ctrl_mode = 3;

cur_tgt = 0.8;         % [A]
spd_tgt = 5000*rpm2rps; % [rad/s]
trq_tgt = 0.0065;        % [Nm]

ctrl_itvl = 0.05;       % [sec] 

spd_init = 500*rpm2rps;  %[rad/s]


% TIME 
dt     = 0.001;         % [sec] time step 
tdur   = 72;          % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [n] step lengths

ang   = zeros(1,tlgth);



out_cur_motor = zeros(1,tlgth);
out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);

% Inital Values
trq = 0;
spd = 0;
t_cur     = 0;
trq_m = 0;
spd_m = spd_init;
spd_m_old = spd_init;
trq_m_old = 0;
tick_tot  = 0;
cur_motor = 0;
spd       = spd_init;

ctrl_next = 0;
m_next = 0;

cur_cmd = 0;
t_m_cur = 0;
t_m_old = 0;

pid_spd.init = 1;
pid_trq.init = 1;

n_filter = 0.5;
interval = 1;

for i = 2:1:tlgth
    
    % Set Measure
    if spd > 4000*rpm2rps
        rds = 4;
    else if spd > 3000*rpm2rps
            rds = 3;
         else if spd > 1500*rpm2rps
                 rds = 2;
             else
                 rds = 1;
             end
        end
    end
    
    m_itvl = 2*pi/spd*rds;

    
    % Back EMF Saturation
    iemf = (V_in-ke*spd)/R; %[V]
    cur_cmd = saturation(cur_cmd,iemf);
    
    % Voltage check / Diode check
    Vemf = ke*spd; % [V] Voltage back emf
    Vmtr = R*cur_motor; % [V] Voltage across motor
    
    if (abs((V_in*sign(cur_cmd)-Vemf-Vmtr)) > abs(V_in*sign(cur_cmd)))
        fprintf('[%.3f] Internal Current\n',tspan(i));
        cur_int   = -1/km*polyval(frct_n,spd); % in intcur ops, this is int current + frict, 
       
    else
        cur_int   = -1/km*polyval(frct_p,spd); % in normal ops, this is friction + bemf
    end
    
    cur_motor = cur_cmd-cur_int;

    trq = km*cur_motor;
    
    spd = spd + trq/J*dt;
    
    % Measurement
if tspan(i)>= m_next 
    m_next = m_next + m_itvl;
    
    t_m_cur = tspan(i);
    
    spd_m = 0.1*spd+0.9*spd_m; 
    trq_m = 0.1*trq+0.9*trq_m;
    
    spd_m_old = spd_m;
    trq_m_old = trq_m;
    t_m_old   = t_m_cur;
end

% Controller Step 
if tspan(i)>= ctrl_next 

    ctrl_next = ctrl_next + ctrl_itvl;
    
    switch ctrl_mode 
        
        case 1
            cur_cmd = cur_tgt;
        case 2
            pid_spd.kp = 100E-3;
            pid_spd.ki = 3.0e-05;
            pid_spd.kd = 0.0;
            cur_cmd = 0;
            cur_cmd = pid_controller(cur_cmd, spd_tgt - spd, pid_spd, ctrl_itvl);
            pid_spd.init = 0;
            
        case 3
            pid_trq.kp = 5.0;
            pid_trq.ki = 0.03;
            pid_trq.kd = 0.0;
            
            % Pre-load
            if (pid_trq.init)
                fprintf('init\n');
                
              if(spd_m*trq_tgt > 0.0)
                  fprintf('(spd(i-1)*trq_tgt>0\n');
                    cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd_m));
              else
                  fprintf('(spd(i-1)*trq_tgt<0\n');
                  trq_f = polyval(frct_n,spd_m);
                  
                  if (abs(trq_tgt) < abs(trq_f))
                      cur_cmd = 0;
                  else
                      cur_cmd = 1/km*(trq_tgt - trq_f);
                  end
              end
              
            else
                cur_cmd = 0.0;
            end
            
            cur_cmd = pid_controller(cur_cmd, trq_tgt - trq_m, pid_trq, dt);
                        
            pid_trq.init = 0;
            

        case 4
            if(spd*trq_tgt>0)
                cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd));
            end
            
        otherwise
    end
end

    
% Maximum allowable current
cur_cmd = saturation(cur_cmd,cur_max);

% Output Read
out_trq_m(i)     = trq_m;
out_spd_m(i)     = spd_m;
out_trq(i)       = trq;
out_spd(i)       = spd;
out_cur_motor(i) = cur_motor;
out_cur_cmd(i)   = cur_cmd;
end

%% PLOT
close all;
figure
plot(tspan,out_trq*1000);
grid on; hold on;
plot(tspan,out_trq_m*1000);
%axis([-inf inf -1.0 10.0 ]);
xlabel('Time [sec');
ylabel('Torque [mNm]');

figure
plot(tspan,out_spd*rps2rpm);
grid on; hold on;
plot(tspan,out_spd_m*rps2rpm);
axis([-inf 72 500 5000]);
xlabel('Time [sec');
ylabel('Speed [rpm]');

figure
plot(tspan,out_cur_cmd*1000);
grid on; hold on;
plot(tspan,out_cur_motor*1000);
legend('cur_c_m_d','cur_m_o_t_o_r');
% axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec');
ylabel('Tgt_Cur [mA]');
