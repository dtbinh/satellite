clear all
clc
format long

rpm2rps = 2*pi/60;

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
R   = 2.3;                % [R] Terminal Resistance
cur_max = 0.9;            % [A] Max Allowable current
cur_min = 0.00; %[A]
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

ctrl_mode = 4;

cur_tgt = 0.8;          % [A]
spd_tgt = 5000*rpm2rps; % [rad/s]
trq_tgt = -0.0065;      % [Nm]

ctrl_itvl = 0.05;       % [sec] 

spd_init = 5000*rpm2rps;  %[rad/s]

% TIME 
dt     = 0.001;         % [sec] time step 
tdur   = 72;          % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [n] step lengths

ang   = zeros(1,tlgth);

rds = 1; % Measurement Rounds

out_cur_motor = zeros(1,tlgth);
out_cur_cmd = zeros(1,tlgth);
out_trq     = zeros(1,tlgth);  % [Nm]
out_spd     = zeros(1,tlgth);
out_trq_m   = zeros(1,tlgth);  % [Nm]
out_spd_m   = zeros(1,tlgth);

% Inital Values
trq = 0;
spd       = spd_init;
t_cur     = 0;
trq_m = 0;
spd_m = spd_init;
spd_m_old = spd_init;
trq_m_old = 0;
tick_tot  = 0;
cur_motor = 0;
m_itvl = 0;

ctrl_next = 0;
m_next = 0;

cur_cmd = 0;
t_m_cur = 0;
t_m_old = 0;

pid_spd.init = 1;
pid_trq.init = 1;
trq_open_init = 1;
n_filter = 0.5;
interval = 1;

% Open Torque Parameters
open_cnt_tot = 0;
open_next = 0;
open_init = 1;
open_rds = 1;
for i = 2:1:tlgth

% --------------- Motor Dynamics-----------------    
    % Back EMF Saturation
    iemf = (V_in-ke*spd)/R; %[V]
    cur_cmd = satcheck(cur_cmd,iemf);
    
    % Voltage check / Diode check
    Vemf = ke*spd; % [V] Voltage back emf
    Vmtr = R*cur_motor; % [V] Voltage across motor
    
    if (abs((V_in*sign(cur_cmd)-Vemf-Vmtr)) > abs(V_in*sign(cur_cmd)))
%         fprintf('[%.3f] Internal Current\n',tspan(i));
        cur_int   = -1/km*polyval(frct_n,spd); % in intcur ops, this is int current + frict, 
       
    else
        cur_int   = -1/km*polyval(frct_p,spd); % in normal ops, this is friction + bemf
    end
    
    cur_motor = cur_cmd-cur_int;

    % Actual Torque
    trq = km*cur_motor;
    
    % Actual Speed 
    spd = spd + trq/J*dt;
    
% --------------- Measurement ------------------
if tspan(i)>= m_next 

    % Set Measure
    m_itvl = 2*pi/spd*rds;
        
    m_next = m_next + m_itvl;
        
    % Update Time Stamp for Old 
    t_m_old   = t_m_cur;
    spd_m_old = spd_m;
    
    % Update Current Speed Measurement
    t_m_cur = tspan(i);
    spd_m = 0.1*spd+0.9*spd_m; 
%     fprintf('spd_m: %.6f t_m_cur: %.6f m_itvl: %.6f\n',spd_m,t_m_cur, m_itvl);
    
end

% --------------- Controller Step ------------------ 
if tspan(i)>= ctrl_next 

    ctrl_next = ctrl_next + ctrl_itvl;
    
    if t_m_old < t_m_cur
        trq_m_old = trq_m;
        spd_err = (spd_m - spd_m_old);
        trq_dt = t_m_cur - t_m_old;
        
        trq_m = J*(spd_m - spd_m_old)/(t_m_cur - t_m_old);
%         fprintf('trq_m: %.6f spd_err: %.6f trq_dt: %.6f\n', trq_m, spd_err, trq_dt);
    else
        fprintf('huh?\n');
    end
    
    
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
                      cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd_m));
                  end
              end
              
            else
                cur_cmd = 0.0;
            end
            

            cur_cmd = pid_controller(cur_cmd, trq_tgt - trq_m, pid_trq, ctrl_itvl);
                        
            pid_trq.init = 0;
            
%         case 4
%             cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd));
%             
        otherwise
            
    end
end



% --------------- Open Loop Control ------------------
if tspan(i)>= open_next
    
    if (ctrl_mode == 4)
        
        % Set Interval
        open_itvl = 2*pi/spd*open_rds;
        fprintf('open_itvl: %.6f rpm:%.6f\n',open_itvl,spd*rps2rpm);
        open_next = open_next+open_itvl;
        open_cnt_tot = open_cnt_tot+1;

        % Set Minimum Count
        if spd>4000*rpm2rps
            cnt_min = 10;
        elseif spd>3000*rpm2rps
            cnt_min = 7;
        elseif spd>2000*rpm2rps
            cnt_min = 4;
        elseif spd>1000*rpm2rps
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
         fprintf('tot: %d ps: %d p1:%d  p2:%d\n',open_cnt_tot,cnt_ps, cnt_p1,cnt_p2);
        if((p_cnt_old + cnt_ps) <= open_cnt_tot)
%             fprintf('-----------------cnt_ps----------------\n');

            if(spd_m*trq_tgt>0)
                
                cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd));
                p1_ready = 1;   
                cnt_p1 = cnt_min;
                cnt_p2 = cnt_min;
                cnt_ps = 2*cnt_min;
                
            else
                % Calculate the Internal Current
                trq_int = polyval(frct_n,spd_m);

                if(abs(trq_int) > abs(trq_tgt))
                    
                    % Drive the minimum reverse current in part 1 (triggers internal current)
                    if(spd_m > 0.0)  
                        cur_cmd = -cur_min; 
                    else
                        cur_cmd = +cur_min; 
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
                    cur_fw = update_cur_fw(trq_tgt,trq_int, cnt_p1, cnt_p2, cnt_ps, ki, frct_p, spd_m);
    
                    % Check cur_fw
                    while(abs(cur_fw) > cur_max)
                        if(cnt_p1 > cnt_min)
                            cnt_p1 = cnt_p1 - 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        else
                            cnt_p2 = cnt_p2 + 1;
                            cnt_ps = cnt_p1 + cnt_p2;
                        end
                    
                        cur_fw = update_cur_fw(trq_tgt,trq_int, cnt_p1, cnt_p2, cnt_ps, ki, frct_p, spd_m);
                    end
                    
                    p1_ready = 0;
                else
                    cur_cmd = 1/km*(trq_tgt - polyval(frct_p,spd));
                    p1_ready = 1;   
                    cnt_p1 = cnt_min;
                    cnt_p2 = cnt_min;
                    cnt_ps = 2*cnt_min;
                    
                end

            end
            %Set Current
            
            
            
            % Update Count Set
            p_cnt_old = p_cnt_old + cnt_ps;
            
%         fprintf('new tot: %d ps: %d p1:%d  p2:%d\n',open_cnt_tot,cnt_ps, cnt_p1,cnt_p2);    
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
cur_cmd = satcheck(cur_cmd,cur_max);

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
legend('trq','trq_{meas}');
%axis([-inf inf -1.0 10.0 ]);
xlabel('Time [sec]');
ylabel('Torque [mNm]');
title('Torque [mNm] vs Time [sec]');

figure
plot(tspan,out_spd*rps2rpm);
grid on; hold on;
plot(tspan,out_spd_m*rps2rpm);
axis([-inf inf 400 5100]);
xlabel('Time [sec]');
ylabel('Speed [rpm]');
title('Speed [rpm] vs Time [sec]');

figure
plot(tspan,out_cur_cmd*1000);
grid on; hold on;
plot(tspan,out_cur_motor*1000);
legend('cur_c_m_d','cur_m_o_t_o_r');
% axis([-inf inf -1000.0 1000.0 ]);
xlabel('Time [sec]');
ylabel('Current [mA]');
title('Current [mA] vs Time [sec]');