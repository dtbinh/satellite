function out = pid_controller(out, in, pid, dt)

persistent sum in_old;


sum = sum + in*dt;

if (pid.init==1)
    fprintf('pid init\n');
    if(pid.ki ~= 0.0)
        sum = out/pid.ki; 
    else
        sum = 0.0;
    end
    in_old = in;
end


out = pid.kp*in + pid.ki*sum + pid.kd*(in-in_old)/dt;

in_old = in;

end