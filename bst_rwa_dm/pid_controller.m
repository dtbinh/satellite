function out = pid_controller(kp,ki,kd,in,dt)

persistent sum in_old
if isempty(sum)
    sum = 0;
    in_old = 0;
    
end
sum = (sum + in)*dt;


out = kp*in + ki*sum + kd*(in-in_old)/dt;
in_old = in;
end