function out = pid_controller(out, in, pid, dt)

persistent sum in_old init;

if isempty(sum)
    sum = 0;
    in_old = 0;
    init = pid.init;
end

sum = sum + in*dt;

if (init)
    fprintf('pid init\n');
    if(pid.ki ~= 0.0)
        sum = out/pid.ki; 
    else
        pid.sum = 0.0;
    end
        init = 0; 
end

out = pid.kp*in + pid.ki*sum + pid.kd*(in-in_old)/dt;
in_old = in;
end