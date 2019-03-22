% ------------------------------------------------------------------------
% This Function represents the wheel drive electronics of the reaction
% wheel.
% ------------------------------------------------------------------------
function ctrl = rw_wde(par, dat, ctrl)

switch ctrl.ctrl_mode 
    % Current control
    case 1
        ctrl.cur_cmd = ctrl.cur_tgt;
    
    % Speed control
    case 2
        ctrl.cur_cmd = 0.0;
        ctrl.pid_spd = rw_pid_controller(ctrl.cur_cmd, ctrl.spd_tgt - dat.spd_m, ctrl.pid_spd, ctrl.ctrl_itvl);
        ctrl.cur_cmd = ctrl.pid_spd.out;
        ctrl.pid_spd.init = 0;
    
    % Torque control
    case 3

        % Pre-load
        if (ctrl.pid_trq.init)
            
          frctn_p =  rw_get_friction(par, dat.spd_m, dat.temp, 'pos'); % [Nm]
          cur_nom = (ctrl.trq_tgt - frctn_p)/par.km_nom;
          current = (ctrl.trq_tgt - frctn_p)/rw_get_km(par, dat.spd_m, cur_nom);

          if((dat.spd_m*ctrl.trq_tgt <= 0.0)&&(abs(ctrl.trq_tgt)>abs(frctn_p)))

             cur_int = rw_get_cur_int(par, dat.spd_m, current, dat.temp, 0);
            
             if(abs(cur_int) >= 0.0)
                 current = 0.0;
             end
          else
              cur_int = 0.0;
          end
            ctrl.cur_cmd = current;
            
             fprintf('Pre-load cur_int:%9.6f  Current: %.6f frctn_p: %.6f km:%.6f\n',cur_int, current, frctn_p, rw_get_km(par, dat.spd_m, cur_nom));
            
        else
            ctrl.cur_cmd = 0.0;
        end
        fprintf('Torque pid\n');
        ctrl.pid_trq = rw_pid_controller(ctrl.cur_cmd, ctrl.trq_tgt - dat.trq_m, ctrl.pid_trq, ctrl.ctrl_itvl*1000);
        ctrl.cur_cmd = ctrl.pid_trq.out;            
        ctrl.pid_trq.init = 0;

    otherwise

end



end