
% 5000 rpm - 83.33 rounds per sec - 0.012 sec/round - 1 msec is 0.083 rds
% 4000 rpm - 66.67 rounds per sec - 0.015 sec/round - 1 msec is 0.067 rds
% 3000 rpm - 50.00 rounds per sec - 0.020 sec/round - 1 msec is 0.050 rds
% 2000 rpm - 33.33 rounds per sec - 0.030 sec/round
% 1000 rpm - 16.67 rounds per sec - 0.060 sec/round
% 500  rpm -  8.33 rounds per sec - 0.120 sec/round

function [trq, spd, trq_m, spd_m, cur_cmd,trq_gross, frct,km_exp,cur_int] = rw_model(model,ctrl_mode, cur_tgt, spd_tgt, trq_tgt, vol_in, spd_init, dt, tnow, temp) 
rps2rpm = 60/2/pi;
tt = tnow; % Running Time in Reaction Wheel

persistent init m_next ctrl_next rw_par rw_dat rw_ctrl
% fprintf('rw run time:%.3f  dt:%.3f\n',tt,dt);

if tnow == 0
    fprintf('Reaction Wheel Initialisation: %s\n',model);

    [rw_par, rw_dat, rw_ctrl] = rw_init(spd_init, temp, model);
    
    % Initiate Speed Measurement Interval
    if (rw_dat.spd ~=0)
        m_itvl = 2*pi/abs(rw_dat.spd)* rw_par.rds;
        m_next = m_itvl;
    else
        m_itvl = 1.0;
        m_next = m_itvl;
    end
    
    ctrl_next = 0;


    init = 1;

    
    fprintf('Reaction Wheel Model Running...\n');
else 
    if init ~= 1
        fprintf('Reaction Wheel Not Initialised\n');
        return
    end
end 
        
        
% User Input
rw_ctrl.ctrl_mode = ctrl_mode; % [] Controller Mode
rw_ctrl.cur_tgt = cur_tgt;     % [A]
rw_ctrl.spd_tgt = spd_tgt;     % [rad/s]
rw_ctrl.trq_tgt = trq_tgt;     % [Nm]
rw_dat.temp     = temp;        % [K]
     fprintf('\n[%9.6f] open t_next:%9.6f ===========================\n',tt,rw_ctrl.open_trq.t_next);
     fprintf('spd: %9.6f spd_m: %.6f\n',rw_dat.spd*rps2rpm,rw_dat.spd_m*rps2rpm);
        % --------------- rw_par dynamics ------------------
        [rw_dat.trq,rw_dat.spd,rw_dat.frct,rw_dat.cur_int] = rw_dynamics(rw_par,vol_in,rw_ctrl.cur_cmd, rw_dat.spd, rw_dat.temp, dt);
        
        % --------------- Measurement ------------------
        if tt>= m_next 
            % Set Measure
            m_itvl = 2*pi/abs(rw_dat.spd)* rw_par.rds;
            m_next = m_next + m_itvl;

            rw_dat = rw_hall(rw_par, rw_dat, rw_dat.spd, tt);
            
             % Torque Measurement
             rw_dat.trq_m =  rw_par.J*(rw_dat.spd_m - rw_dat.spd_m_old)/(rw_dat.t_spd_m_cur - rw_dat.t_spd_m_old);
                
            fprintf('trq_m: %9.6f dspd:%9.6f dt: %9.6f\n',rw_dat.trq_m,rw_dat.spd_m - rw_dat.spd_m_old, rw_dat.t_spd_m_cur - rw_dat.t_spd_m_old  );

            if (rw_dat.spd_m*rw_ctrl.cur_cmd>0.0)
                frct = rw_get_friction(rw_par, rw_dat.spd_m, temp, 'pos');
            %     fprintf('Positive Friction: %.9f [Nm]\n',frct);
            elseif(rw_dat.spd_m*rw_ctrl.cur_cmd<0.0)
                frct = rw_get_friction(rw_par, rw_dat.spd_m, temp, 'neg');
            %     fprintf('Negative Friction: %.9f [Nm]\n',frct);
            else
                frct = rw_get_friction(rw_par, rw_dat.spd_m, temp, 'idle');
            %     fprintf('Idle Friction: %.9f [Nm]\n',frct);
            end
            rw_dat.trq_gross = rw_dat.trq_m - frct;

    %        fprintf('rw_dat.trq_m: %.6f spd_err: %.6f trq_dt: %.6f\n', rw_dat.trq_m, spd_err, trq_dt);
           

           
        end

        % --------------- Controller Step ------------------ 
        if tt>= ctrl_next 
            ctrl_next = ctrl_next + rw_ctrl.ctrl_itvl;
            
            % Wheel Drive Electronics
            rw_ctrl = rw_wde(rw_par, rw_dat, rw_ctrl);
        end

        % --------------- Open Loop Control ------------------
        if (rw_ctrl.ctrl_mode == 4)
            if tt>= rw_ctrl.open_trq.t_next

                % Set Interval
                open_itvl = 2*pi/abs(rw_dat.spd)*rw_ctrl.open_trq.rds;
                rw_ctrl.open_trq.t_next = rw_ctrl.open_trq.t_next+open_itvl;
                rw_ctrl.open_trq.cnt_tot = rw_ctrl.open_trq.cnt_tot+1;

        % fprintf('open_itvl: %.6f rpm:%.6f\n',open_itvl,spd*rps2rpm);

                % Open Loop Control
                rw_ctrl = rw_open(rw_par, rw_dat, rw_ctrl);



            end
        end

        % Maximum allowable current
        rw_ctrl.cur_cmd = rw_satcheck(rw_ctrl.cur_cmd,rw_par.cur_max);

        % Maximum allowable speed
        rw_ctrl.cur_cmd = rw_satzero(rw_ctrl.cur_cmd,rw_dat.spd_m,rw_par.spd_max);

% fprintf('cur_cmd: %.6f\n',rw_ctrl.cur_cmd);

km_exp = rw_get_km(rw_par, rw_dat.spd_m, rw_ctrl.cur_cmd);

trq     = rw_dat.trq;
spd     = rw_dat.spd;
trq_m   = rw_dat.trq_m;
spd_m   = rw_dat.spd_m;
cur_cmd = rw_ctrl.cur_cmd;
trq_gross = rw_dat.trq_gross;
frct = rw_dat.frct;
cur_int = rw_dat.cur_int;
end