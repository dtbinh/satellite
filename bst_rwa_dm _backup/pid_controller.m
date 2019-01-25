function pid = pid_controller(out, in, pid, dt)

persistent in_old;

pid.sum = pid.sum + in*dt;

% Initialisation
if (pid.init==1)
%     fprintf('pid init\n');
    
    if(pid.ki ~= 0.0)
%         fprintf('pid integrator init\n');
        pid.sum = out/pid.ki; 
    else
        pid.sum = 0.0;
    end
    
    in_old = in;
    pid.init = 0;
end

% Anti Windup Implement
if (pid.aw_flg)
    fprintf('ANTI WIND UP!\n');
    pid.sum = pid.sum*pid.aw_fact;  
end

% PID Controller
pid.out = pid.kp*in + pid.ki*pid.sum + pid.kd*(in-in_old)/dt;

% Anti Windup Check
if(pid.aw_thr ~=0.0)
    if(pid.out > pid.aw_thr)

          pid.out = pid.aw_thr;
          pid.aw_flg = 1;

    elseif(pid.out < -pid.aw_thr)

          pid.out = -pid.aw_thr;
          pid.aw_flg = 1;
    end
end

in_old = in;

end