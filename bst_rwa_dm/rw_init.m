function [par, dat, ctrl] = rw_init(spd_init)

rpm2rps = 2*pi/60;

% ---- Inital Values ------
        dat.trq    = 0;        % [Nm] Instanteneous Torque
        dat.spd    = spd_init; % [Nm] Instanteneous Speed
        dat.cur_motor = 0;

        dat.trq_m  = 0;
        dat.spd_m  = spd_init;
        dat.spd_m_old = spd_init;
        dat.t_spd_m_cur = 0;
        dat.t_spd_m_old = 0;
        
        
        % --------------- Motor Parameters ---------------
        par.ki  = 96.154;             % [A/Nm] Current Constant
        par.km  = 1/96.154;         % [Nm/A] Torque Constant
        par.ke  = 1.026E-3/(2*pi/60); % [V/(rad/s)] Back-emf constant
        par.J   = 978.548e-6;         % [kgm2] Moment of Inertia
        par.R   = 2.3;                % [R] Terminal Resistance
        par.spd_max = 5050.0*rpm2rps; % [rad/s] Maximum wheel Speed
        par.cur_max = 0.9;            % [A] Max Allowable current
        par.cur_min = 0.001;           %[A] This value has to be zero (cur_min = 0.001 in par)

        par.filter = 0.9; % Complimentary Fitler (0.0 - no filter, 1.0 full filter)
        par.rds    = 1;   % Rounds when Speed is Updated 

        % Friction Model
        par.frct_c = [-3.77e-09, -1.42e-06, -6.76e-04];
        par.frct_p = [-3.39e-09, -1.57e-06, -5.89e-04];
        par.frct_n = [-9.44e-09, -4.31e-05, +3.57e-04];

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

        ctrl.ctrl_itvl = 0.05;       % [sec] 
        ctrl.cur_cmd = 0;
        ctrl.pid_spd = pid_spd;
        ctrl.pid_trq = pid_trq;
        ctrl.open_trq = open_trq;


end