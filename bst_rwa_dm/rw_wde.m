function ctrl = rw_wde(par, dat, ctrl)

switch ctrl.ctrl_mode 
    case 1
        ctrl.cur_cmd = ctrl.cur_tgt;
    case 2
        ctrl.cur_cmd = 0.0;
        ctrl.pid_spd = rw_pid_controller(ctrl.cur_cmd, ctrl.spd_tgt - dat.spd_m, ctrl.pid_spd, ctrl.ctrl_itvl);
        ctrl.cur_cmd = ctrl.pid_spd.out;
        ctrl.pid_spd.init = 0;

% fprintf('[%.3f] ctrl.cur_cmd: %.3f [A] spd err: %9.6f [A] sum:  %9.6f dat.spd_m:%.3f [rpm]\n', ...
%          tspan(i), ctrl.cur_cmd, ctrl.pid_spd.kp*(ctrl.spd_tgt - dat.spd_m), ctrl.pid_spd.ki*ctrl.pid_spd.sum, dat.spd_m/rpm2rps);

    case 3

        % Pre-load
        if (ctrl.pid_trq.init)

          if(dat.spd_m*ctrl.trq_tgt > 0.0)

                ctrl.cur_cmd =  par.ki*(ctrl.trq_tgt - rw_get_friction(par,dat.spd_m,'pos'));

          else

              trq_f = rw_get_friction(par,dat.spd_m,'neg');

              if (abs(ctrl.trq_tgt) < abs(trq_f))
                  ctrl.cur_cmd = 0;
              else
                  ctrl.cur_cmd =  par.ki*(ctrl.trq_tgt - rw_get_friction(par,dat.spd_m,'pos'));
              end
          end

        else
            ctrl.cur_cmd = 0.0;
        end

        ctrl.pid_trq = rw_pid_controller(ctrl.cur_cmd, ctrl.trq_tgt - dat.trq_m, ctrl.pid_trq, ctrl.ctrl_itvl*1000);
        ctrl.cur_cmd = ctrl.pid_trq.out;            
        ctrl.pid_trq.init = 0;

% fprintf('[%.3f] ctrl.cur_cmd: %.3f [A] trq err: %9.6f [Nm] sum:  %9.6f dat.spd_m:%.3f [rad/s]\n', ...
%           tspan(i), ctrl.cur_cmd, ctrl.pid_trq.kp*(ctrl.trq_tgt - dat.trq_m), ctrl.pid_trq.ki*ctrl.pid_trq.sum, dat.spd_m);

    otherwise

end



end