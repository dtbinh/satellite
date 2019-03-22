function [par, dat, ctrl] = rw_init(spd_init,temp_init, model)

rpm2rps = 2*pi/60;

kp_cur =     (((0.56+0.1) * 8192.0) / 0.698);   % out[DAC] = I[A]*kp[DAC/A] <<>> 0.56Ohm Shunt + 0.1 Ohm Wire Resistance // 12 bit // Reference voltage 0.698 Volt

% ---- Inital Values ------
        dat.trq    = 0;        % [Nm] Instanteneous Torque
        dat.spd    = spd_init; % [Nm] Instanteneous Speed
        dat.cur_motor = 0;
        
        dat.temp   = temp_init;
        dat.trq_m  = 0;
        dat.spd_m  = spd_init;
        dat.spd_m_old = spd_init;
        dat.t_spd_m_cur = 0;
        dat.t_spd_m_old = 0;
        dat.trq_gross  = 0;
        
        
        % Friction Model
        switch model
            case 'dm'
                par.frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
                par.frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
                par.frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];
            case'deorbit_fm1'
                        
                % --------------- Motor Parameters ---------------
                par.ki  = 96.154;             % [A/Nm] Current Constant
                par.ke  = 1.026E-3/rpm2rps;  % [V/(rad/s)] Back-emf constant
                par.J   = 978.548e-6;          % [kgm2] Moment of Inertia
                par.r_mot  = 2.00;             % [Ohm] Terminal Resistance
                par.r_mot_temp = 0.01;         % [Ohm/degC] Temperature constant
                par.spd_max    = 5050.0*rpm2rps; % [rad/s] Maximum wheel Speed
                par.cur_max    = 0.9;          % [A] Max Allowable current
                par.cur_min    = 0.001;        %[A] This value has to be zero (cur_min = 0.001 in par)
                par.diode_drop = 0.250;        % [V]
                par.filter     = 0.9;          % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.rds        = 1;            % Rounds when Speed is Updated 
                par.temp_ref   = 18.0;         % [degC]
                par.temp_mot_a = 1.0;
                
                par.km     = 10.45e-3;                      % [Nm/A] Torque Constant Modelling
                par.km_nom = 10.0e-3;                       % [Nm/A] Torque Constant Nominal
                par.cur_km = [0.3, 0.6, par.cur_max];
                par.km_val = [ 11.9e-3, 10.2e-3, ...               
                               13.4e-3, 10.4e-3, ...
                               13.4e-3, 10.4e-3 ];
                par.km_spd = [1250.0*rpm2rps, 2500.0*rpm2rps,...
                               500.0*rpm2rps, 1800.0*rpm2rps,...
                               500.0*rpm2rps, 1200.0*rpm2rps];
                par.frct_c = [-1.96e-09, -1.75e-06, -6.81e-04];
                par.frct_p = [-1.66e-09, -1.89e-06, -6.39e-04];
                par.frct_n = [-8.23e-09, -4.92e-05,  5.59e-04];
                par.frctn_temp_p = [-0.0001436e-3, 0.0056346e-3];
                par.kp_cur = kp_cur;
                
                % Controller
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

                % Torque Open Parameters
                open_trq.cnt_tot = 0;
                open_trq.t_next = 0;
                open_trq.init = 1;
                open_trq.rds  = 1;

            case'deorbit_fm2'
                
                % --------------- Motor Parameters ---------------
                par.ki  = 96.154;             % [A/Nm] Current Constant
                par.ke  = 1.026E-3/rpm2rps;  % [V/(rad/s)] Back-emf constant
                par.J   = 978.548e-6;          % [kgm2] Moment of Inertia
                par.r_mot  = 2.00;             % [Ohm] Terminal Resistance
                par.r_mot_temp = 0.01;         % [Ohm/degC] Temperature constant
                par.spd_max    = 5050.0*rpm2rps; % [rad/s] Maximum wheel Speed
                par.cur_max    = 0.9;          % [A] Max Allowable current
                par.cur_min    = 0.001;        %[A] This value has to be zero (cur_min = 0.001 in par)
                par.diode_drop = 0.250;        % [V]
                par.filter     = 0.9;          % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.rds        = 1;            % Rounds when Speed is Updated 
                par.temp_ref   = 18.0;         % [degC]
                par.temp_mot_a = 1.0;
                
                par.km  = 10.35e-3;           % [Nm/A] Torque Constant Modelling
                par.km_nom  = 10.0e-3;        % [Nm/A] Torque Constant Nominal
                par.cur_km = [0.3, 0.6, par.cur_max];
                par.km_val = [ 11.8e-3, 10.0e-3, ...               
                               13.4e-3, 10.2e-3, ...
                               13.4e-3, 10.2e-3 ];
                par.km_spd = [1250.0*rpm2rps, 2500.0*rpm2rps,...
                               400.0*rpm2rps, 1900.0*rpm2rps,...
                               400.0*rpm2rps, 1400.0*rpm2rps];
                par.frct_c = [-2.07e-09, -1.65e-06, -6.93e-04];
                par.frct_p = [-2.17e-09, -1.61e-06, -6.87e-04];
                par.frct_n = [-7.60e-09, -4.78e-05,  5.46e-04];
                par.frctn_temp_p = [-0.0001436e-3, 0.0056346e-3];
                par.kp_cur = kp_cur*800.0/788.0;


                % Controller
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

                % Torque Open Parameters
                open_trq.cnt_tot = 0;
                open_trq.t_next = 0;
                open_trq.init = 1;
                open_trq.rds  = 1;

            case'deorbit_fm3'
                
                % --------------- Motor Parameters ---------------
                par.ki  = 96.154;             % [A/Nm] Current Constant
                par.ke  = 1.026E-3/rpm2rps;  % [V/(rad/s)] Back-emf constant
                par.J   = 978.548e-6;          % [kgm2] Moment of Inertia
                par.r_mot  = 2.00;             % [Ohm] Terminal Resistance
                par.r_mot_temp = 0.01;         % [Ohm/degC] Temperature constant
                par.spd_max    = 5050.0*rpm2rps; % [rad/s] Maximum wheel Speed
                par.cur_max    = 0.9;          % [A] Max Allowable current
                par.cur_min    = 0.001;        %[A] This value has to be zero (cur_min = 0.001 in par)
                par.diode_drop = 0.250;        % [V]
                par.filter     = 0.9;          % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.rds        = 1;            % Rounds when Speed is Updated 
                par.temp_ref   = 18.0;         % [degC]
                par.temp_mot_a = 1.0;
                
                par.km  = 1/96.154;           % [Nm/A] Torque Constant Modelling
                par.km_nom  = 10.0e-3;        % [Nm/A] Torque Constant Nominal
                par.cur_km = [0.3, 0.6, par.cur_max];
                par.km_val = [ 12.2e-3, 10.4e-3, ...               
                               13.8e-3, 10.4e-3, ...
                               13.7e-3, 10.5e-3 ];
                par.km_spd = [1250.0*rpm2rps, 2500.0*rpm2rps,...
                               500.0*rpm2rps, 1800.0*rpm2rps,...
                               500.0*rpm2rps, 1200.0*rpm2rps];
                par.frct_c = [-1.88e-09, -1.82e-06, -6.65e-04];
                par.frct_p = [-1.88e-09, -1.81e-06, -6.47e-04];
                par.frct_n = [-7.20e-09, -5.01e-05,  6.25e-04];
                par.frctn_temp_p = [-0.0001436e-3, 0.0056346e-3];
                par.kp_cur = kp_cur*800.0/789.0;

                % Controller
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

                % Torque Open Parameters
                open_trq.cnt_tot = 0;
                open_trq.t_next = 0;
                open_trq.init = 1;
                open_trq.rds  = 1;

            case'nexsat_fm3'
                par.ki         = 1/20.2e-3;         % [A/Nm] Current Constant
                par.ke         = 2.115E-3/rpm2rps;  % [V/(rad/s)] Back-emf constant
                par.J          = 978.548e-6;        % [kgm2] Moment of Inertia
                par.r_mot      = 8.40;              % [Ohm] Terminal Resistance
                par.r_mot_temp = 0.01;              % [Ohm/degC] Temperature constant
                par.spd_max    = 5050.0*rpm2rps;    % [rad/s] Maximum wheel Speed
                par.cur_max    = 0.9;               % [A] Max Allowable current
                par.cur_min    = 0.001;             % [A] This value has to be 0.001 (cur_min = 0.001 in par)
                par.diode_drop = 0.250;             % [V]
                par.filter     = 0.2;               % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.filter_trq = 0.75;              % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.rds        = 1;                 % Rounds when Speed is Updated 
                par.temp_ref   = 18.0;              % [degC]
                par.temp_mot_a = 1.0;
                
                par.km         = 20.2e-3;               % [Nm/A] Torque Constant Modelling
                par.km_nom     = 20.2e-3;               % [Nm/A] Torque Constant Nominal
                par.cur_km     = [0.3, 0.6, par.cur_max];
                par.km_val     = [ 12.2e-3, 10.4e-3, ...               
                                   13.8e-3, 10.4e-3, ...
                                   13.7e-3, 10.5e-3 ];
                par.km_spd     = [   0.0*rpm2rps, 5000.0*rpm2rps,...
                                     0.0*rpm2rps, 4000.0*rpm2rps,...
                                  4000.0*rpm2rps, 5000.0*rpm2rps];
                par.frct_c = [-1.33e-09, -2.12e-06, -6.55e-04];
                par.frct_p = [-1.39e-09, -2.13e-06, -5.14e-04];
                par.frct_n = [-1.57e-08, -4.07e-05,  1.83e-04];
                par.frctn_temp_p = [-0.0001436e-3, 0.0056346e-3];
                par.kp_cur = kp_cur*800.0/789.0; 
                
                
                
                % Speed PID Parameter
                pid_spd.init = 1;
                pid_spd.kp   = 65E-3;
                pid_spd.ki   = 65*0.0000001;
                pid_spd.kd   = 0.0;
                pid_spd.sum  = 0.0;
                pid_spd.aw_thr = 0;
                pid_spd.aw_flg = 0;
                pid_spd.aw_fact = 0.6;

                % Torque PID Parameter
                pid_trq.init = 1;
                pid_trq.kp   = 70.0;
                pid_trq.ki   = 0.05;
                pid_trq.kd   = 0.0;
                pid_trq.sum  = 0.0;
                pid_trq.aw_thr = 0;
                pid_trq.aw_flg = 0;
                pid_trq.aw_fact = 1.0;
                
                % Torque Open Parameters
                open_trq.cnt_tot = 0;
                open_trq.t_next = 0;
                open_trq.init = 1;
                open_trq.rds  = 1;
                
            case'nexsat_fm4'
                par.ki         = 1/20.2e-3;         % [A/Nm] Current Constant
                par.ke         = 2.115E-3/rpm2rps;  % [V/(rad/s)] Back-emf constant
                par.J          = 978.548e-6;        % [kgm2] Moment of Inertia
                par.r_mot      = 8.40;              % [Ohm] Terminal Resistance
                par.r_mot_temp = 0.01;              % [Ohm/degC] Temperature constant
                par.spd_max    = 5050.0*rpm2rps;    % [rad/s] Maximum wheel Speed
                par.cur_max    = 0.9;               % [A] Max Allowable current
                par.cur_min    = 0.001;             % [A] This value has to be 0.001 (cur_min = 0.001 in par)
                par.diode_drop = 0.250;             % [V]
                par.filter     = 0.2;               % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.filter_trq = 0.75;              % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
                par.rds        = 1;                 % Rounds when Speed is Updated 
                par.temp_ref   = 18.0;              % [degC]
                par.temp_mot_a = 1.0;
                
                par.km         = 20.2e-3;               % [Nm/A] Torque Constant Modelling
                par.km_nom     = 20.2e-3;               % [Nm/A] Torque Constant Nominal
                par.cur_km     = [0.3, 0.6, par.cur_max];
                par.km_val     = [ 12.2e-3, 10.4e-3, ...               
                                   13.8e-3, 10.4e-3, ...
                                   13.7e-3, 10.5e-3 ];
                par.km_spd     = [   0.0*rpm2rps, 5000.0*rpm2rps,...
                                     0.0*rpm2rps, 4000.0*rpm2rps,...
                                  4000.0*rpm2rps, 5000.0*rpm2rps];
                par.frct_c = [-1.33e-09, -2.12e-06, -6.55e-04];
                par.frct_p = [-1.39e-09, -2.13e-06, -5.14e-04];
                par.frct_n = [-1.57e-08, -4.07e-05,  1.83e-04];
                par.frctn_temp_p = [-0.0001436e-3, 0.0056346e-3];
                par.kp_cur = kp_cur*800.0/789.0; 
                
                
                
                % Speed PID Parameter
                pid_spd.init = 1;
                pid_spd.kp   = 65E-3;
                pid_spd.ki   = 65*0.0000001;
                pid_spd.kd   = 0.0;
                pid_spd.sum  = 0.0;
                pid_spd.aw_thr = 0;
                pid_spd.aw_flg = 0;
                pid_spd.aw_fact = 0.6;

                % Torque PID Parameter
                pid_trq.init = 1;
                pid_trq.kp   = 70.0;
                pid_trq.ki   = 0.05;
                pid_trq.kd   = 0.0;
                pid_trq.sum  = 0.0;
                pid_trq.aw_thr = 0;
                pid_trq.aw_flg = 0;
                pid_trq.aw_fact = 1.0;
                
                % Torque Open Parameters
                open_trq.cnt_tot = 0;
                open_trq.t_next = 0;
                open_trq.init = 1;
                open_trq.rds  = 1;
            otherwise
        end
        
        ctrl.ctrl_itvl = 0.050;       % [sec] 
        ctrl.cur_cmd = 0;
        ctrl.pid_spd = pid_spd;
        ctrl.pid_trq = pid_trq;
        ctrl.open_trq = open_trq;


end