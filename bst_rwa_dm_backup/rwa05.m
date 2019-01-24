clear all
clc
format long
global rw_par rw_dat pid_spd pid_trq
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
vol_in  = 8.0;            % [V] Input Voltage
ctrl_mode = 4;

cur_tgt = 0.8;          % [A]
spd_tgt = 4000*rpm2rps; % [rad/s]
trq_tgt = 0.0065;       % [Nm]

spd_init = 500*rpm2rps;  %[rad/s] Initial Speed

%% SIMULATION  PARAMETERS 
dt     = 0.001;         % [sec] time step 
tdur   = 72.0;          % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [n] step lengths

out_cur_motor = zeros(1,tlgth);
out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);

% Inital Values
trq    = 0;        % [Nm] Instanteneous Torque
spd    = spd_init; % [Nm] Instanteneous Speed
cur_motor = 0;

rw_dat.trq_m  = 0;
rw_dat.spd_m  = spd_init;
rw_dat.spd_m_old = spd_init;
tick_tot  = 0;


cur_cmd = 0;
rw_dat.t_spd_m_cur = 0;
rw_dat.t_spd_m_old = 0;

% Speed PID Parameter
pid_spd.init = 1;
pid_spd.kp = 100E-3;
pid_spd.ki = 300*0.0000001;
pid_spd.kd = 0.0;
pid_spd.sum = 0.0;
pid_spd.aw_thr = 0;
pid_spd.aw_flg = 0;
pid_spd.aw_fact = 0.6;

% Torque PID Parameter
pid_trq.init = 1;
pid_trq.kp = 5.0;
pid_trq.ki = 0.03;
pid_trq.kd = 0.0;
pid_trq.sum = 0.0;
pid_trq.aw_thr = 0;
pid_trq.aw_flg = 0;
pid_trq.aw_fact = 1.0;

trq_open_init = 1;
n_filter = 0.5;
interval = 1;

% Open Torque Parameters
open_cnt_tot = 0;
open_next = 0;
open_init = 1;
open_rds  = 1;

m_itvl = 0;
m_next = 0;

ctrl_itvl = 0.05;       % [sec] 
ctrl_next = 0;

% --------------- Motor Parameters ---------------

rw_par.ki  = 96.154;             % [A/Nm] Current Constant
rw_par.km  = 1/rw_par.ki;         % [Nm/A] Torque Constant
rw_par.ke  = 1.026E-3/(2*pi/60); % [V/(rad/s)] Back-emf constant
rw_par.J   = 978.548e-6;         % [kgm2] Moment of Inertia
rw_par.R   = 2.3;                % [R] Terminal Resistance
rw_par.spd_max = 5050.0*rpm2rps; % [rad/s] Maximum wheel Speed
rw_par.cur_max = 0.9;            % [A] Max Allowable current
rw_par.cur_min = 0.001;           %[A] This value has to be zero (cur_min = 0.001 in rw_par)

rw_par.filter = 0.9; % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
rw_par.rds    = 1;   % Rounds when Speed is Updated 

% Friction Model
rw_par.frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
rw_par.frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
rw_par.frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];

for i = 2:1:tlgth
% --------------- rw_par dynamics ------------------
[trq,spd] = rw_dynamics(rw_par,vol_in,cur_cmd, spd, dt);

% --------------- Measurement ------------------
if tspan(i)>= m_next 
    % Set Measure
    m_itvl = 2*pi/abs(spd)* rw_par.rds;
    m_next = m_next + m_itvl;
if(1)

    rw_dat = rw_hall(rw_par, rw_dat, spd, tspan(i));
      
else        
    % Update Time Stamp for Old 
    rw_dat.t_spd_m_old = rw_dat.t_spd_m_cur;
    rw_dat.spd_m_old   = rw_dat.spd_m;
    
    % Update Current Speed Measurement
    rw_dat.t_spd_m_cur = tspan(i);
    rw_dat.spd_m = (1-rw_par.filter)*spd + rw_par.filter*rw_dat.spd_m;
end   
%   fprintf('rw_dat.spd_m: %.6f rw_dat.t_spd_m_cur: %.6f m_itvl: %.6f\n',rw_dat.spd_m,rw_dat.t_spd_m_cur, m_itvl);
end

% --------------- Controller Step ------------------ 
if tspan(i)>= ctrl_next 

    ctrl_next = ctrl_next + ctrl_itvl;
    
    % Torque Measurement
    if rw_dat.t_spd_m_old < rw_dat.t_spd_m_cur
 
        rw_dat.trq_m =  rw_par.J*(rw_dat.spd_m - rw_dat.spd_m_old)/(rw_dat.t_spd_m_cur - rw_dat.t_spd_m_old);
%       fprintf('rw_dat.trq_m: %.6f spd_err: %.6f trq_dt: %.6f\n', rw_dat.trq_m, spd_err, trq_dt);
    end
    
    
    switch ctrl_mode 
        
        case 1
            
            cur_cmd = cur_tgt;
            
        case 2
            
            cur_cmd = 0.0;
            pid_spd = pid_controller(cur_cmd, spd_tgt - rw_dat.spd_m, pid_spd, ctrl_itvl);
            cur_cmd = pid_spd.out;
            pid_spd.init = 0;
            
            fprintf('[%.3f] cur_cmd: %.3f [A] spd err: %9.6f [A] sum:  %9.6f rw_dat.spd_m:%.3f [rpm]\n', ...
                            tspan(i), cur_cmd, pid_spd.kp*(spd_tgt - rw_dat.spd_m), pid_spd.ki*pid_spd.sum, rw_dat.spd_m/rpm2rps);
            
        case 3
            
            % Pre-load
            if (pid_trq.init)
                
                fprintf('init\n');
                
              if(rw_dat.spd_m*trq_tgt > 0.0)
                  
                  fprintf('(spd(i-1)*trq_tgt>0\n');
                    cur_cmd =  rw_par.ki*(trq_tgt - rw_get_friction(rw_par,rw_dat.spd_m,'pos'));
                    
              else
                  fprintf('(spd(i-1)*trq_tgt<0\n');
                  trq_f = rw_get_friction(rw_par,rw_dat.spd_m,'neg');
                  
                  if (abs(trq_tgt) < abs(trq_f))
                      cur_cmd = 0;
                  else
                      cur_cmd =  rw_par.ki*(trq_tgt - rw_get_friction(rw_par,rw_dat.spd_m,'pos'));
                  end
              end
              
            else
                cur_cmd = 0.0;
            end
            

            pid_trq = pid_controller(cur_cmd, trq_tgt - rw_dat.trq_m, pid_trq, ctrl_itvl*1000);
            cur_cmd = pid_trq.out;            
            pid_trq.init = 0;
            
%             fprintf('[%.3f] cur_cmd: %.3f [A] trq err: %9.6f [Nm] sum:  %9.6f rw_dat.spd_m:%.3f [rad/s]\n', ...
%                              tspan(i), cur_cmd, pid_trq.kp*(trq_tgt - rw_dat.trq_m), pid_trq.ki*pid_trq.sum, rw_dat.spd_m);
         
        otherwise
            
    end
end



% --------------- Open Loop Control ------------------
if tspan(i)>= open_next
    
    if (ctrl_mode == 4)
        
        % Set Interval
        open_itvl = 2*pi/abs(spd)*open_rds;
%         fprintf('open_itvl: %.6f rpm:%.6f\n',open_itvl,spd*rps2rpm);
        open_next = open_next+open_itvl;
        open_cnt_tot = open_cnt_tot+1;

        % Set Minimum Count
        if abs(spd)>4000*rpm2rps
            cnt_min = 10;
        elseif abs(spd)>3000*rpm2rps
            cnt_min = 7;
        elseif abs(spd)>2000*rpm2rps
            cnt_min = 4;
        elseif abs(spd)>1000*rpm2rps
            cnt_min = 2;
        else
            cnt_min = 2;
        end    
        

        % Init
        if(open_init)
            
            cnt_p1 = cnt_min;
            cnt_p2 = cnt_min;
            cnt_ps = 2*cnt_min;
            p_cnt_old = open_cnt_tot - cnt_ps;
            open_init = 0;
        end
%          fprintf('tot: %d ps: %d p1:%d  p2:%d\n',open_cnt_tot,cnt_ps, cnt_p1,cnt_p2);
        if((p_cnt_old + cnt_ps) <= open_cnt_tot)
%             fprintf('-----------------cnt_ps----------------\n');

            if(rw_dat.spd_m*trq_tgt>0)
%                 fprintf('[%.4f] Pos Direct Control \n',tspan(i));
                cur_cmd = rw_par.ki*(trq_tgt - rw_get_friction(rw_par,rw_dat.spd_m,'pos'));
                p1_ready = 1;   
                cnt_p1 = cnt_min;
                cnt_p2 = cnt_min;
                cnt_ps = 2*cnt_min;
                
            else
                % Calculate the Internal Current
                trq_int = rw_get_friction(rw_par, rw_dat.spd_m ,'neg');
            
                if(abs(trq_int) > abs(trq_tgt))
                    fprintf('[%.4f] Neg Parts Control trq_int:%.6f trq_tgt:%.6f \n',tspan(i),trq_int,trq_tgt);
                    
                    % Drive the minimum reverse current in part 1 (triggers internal current)
                    if(rw_dat.spd_m > 0.0)  
                        cur_cmd = -rw_par.cur_min; 
                    else
                        cur_cmd = +rw_par.cur_min; 
                    end
                    
                    % Check 
                    while( ((cnt_p1/cnt_ps)*abs(trq_int)) < abs(trq_tgt))
                        if(cnt_p2 > cnt_min)
                            cnt_p2 = cnt_p2 - 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        else
                            cnt_p1 = cnt_p1 + 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        end
                    end
                    
                    % Update cur_fw
                    cur_fw = update_cur_fw(trq_tgt, trq_int, cnt_p1, cnt_p2, cnt_ps, rw_dat.spd_m, rw_par);
    
                    % Check cur_fw
                    while(abs(cur_fw) > rw_par.cur_max)
                        if(cnt_p1 > cnt_min)
                            cnt_p1 = cnt_p1 - 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        else
                            cnt_p2 = cnt_p2 + 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        end
                    
                        cur_fw = update_cur_fw(trq_tgt,trq_int, cnt_p1, cnt_p2, cnt_ps, rw_dat.spd_m, rw_par);
                    end
                    
                    p1_ready = 0;
                else
                    fprintf('[%.4f] Neg Direct Control trq_int:%.6f trq_tgt:%.6f \n',tspan(i),trq_int,trq_tgt);
                    cur_cmd = rw_par.ki*(trq_tgt - rw_get_friction(rw_par,spd,'pos'));
                    p1_ready = 1;   
                    cnt_p1 = cnt_min;
                    cnt_p2 = cnt_min;
                    cnt_ps = 2*cnt_min;
                    
                end

            end
            
            % Update Count Set
            p_cnt_old = p_cnt_old + cnt_ps;
            
         fprintf('new tot: %d ps: %d p1:%d  p2:%d\n',open_cnt_tot,cnt_ps, cnt_p1,cnt_p2);    
        elseif ((p_cnt_old + cnt_p1) <= open_cnt_tot)
          
            if (p1_ready == 0)
                
                cur_cmd = cur_fw;
                p1_ready = 1;
            end
             
        end
    end
end
% fprintf('cur_cmd: %.3f \n',cur_cmd); 


% Maximum allowable current
cur_cmd = satcheck(cur_cmd,rw_par.cur_max);

% Maximum allowable speed
cur_cmd = satzero(cur_cmd,rw_dat.spd_m,rw_par.spd_max);


% Output Read
out_trq_m(i)     = rw_dat.trq_m;
out_spd_m(i)     = rw_dat.spd_m;
out_trq(i)       = trq;
out_spd(i)       = spd;
out_cur_motor(i) = cur_motor;
out_cur_cmd(i)   = cur_cmd;
end

%% PLOT
close all;

screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;
screensetting = [0.25*screenwidth 0.25*screenheight screenwidth*0.5 screenheight*0.59];

% Speed
fig = figure;
set(fig,'Position',screensetting);
plot(tspan,out_spd*rps2rpm);
grid on; hold on;
plot(tspan,out_spd_m*rps2rpm);
axis([0 tspan(end) 500 5000]);
xlabel('Time [sec]');
ylabel('Speed [rpm]');
title('Speed [rpm] vs Time [sec]');


% Torque
fig = figure;
set(fig,'Position',screensetting);
plot(tspan,out_trq*1000);
grid on; hold on;
plot(tspan,out_trq_m*1000);
legend('trq','trq_{meas}');
axis([-inf inf -30.0 10.0 ]);
xlabel('Time [sec]');
ylabel('Torque [mNm]');
title('Torque [mNm] vs Time [sec]');


% Current
fig = figure;
set(fig,'Position',screensetting);
plot(tspan,out_cur_cmd*1000);
grid on; hold on;
legend('cur_c_m_d');
axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec]');
ylabel('Current [mA]');
title('Current [mA] vs Time [sec]');


