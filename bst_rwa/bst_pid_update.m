function out = bst_pid_update(out, in, pid, dt)

persistent sum in_old init

if isempty(sum)
   sum    = 0; 
   in_old = 0;
   init   = pid.init;
end

sum = sum + in*dt;
     
% PID Init Command
if (init == 1)
    fprintf('PID Parameters\n');
    fprintf('kp  = %.4f \nki  = %.4f \nkd  = %.4f\n',pid.kp,pid.ki, pid.kd);
    fprintf('init = %.4f\n',pid.init);
    if(pid.ki ~= 0.0)
        sum = out/pid.ki;
        
    else
        pid.sum = 0.0;
    end
    
        init = 0; 
end
  
% Execute anti-windup if flagged
if (pid.aw_flg)
    fprintf('aw_flg \n');
   sum = sum * pid.aw_fact;
end

% PID 
out = pid.kp*in  +  pid.ki*sum  +  pid.kd*((in - in_old)/dt);

            
% Anti windup check
pid.aw_flg = 0;
if(pid.aw_thr ~= 0.0)
    if(out > pid.aw_thr)

            out = pid.aw_thr;
            pid.aw_flg = 1;

    else if(out < -pid.aw_thr)

            out = -pid.aw_thr;
            pid.aw_flg = 1;
         end
    end
end
        
in_old = in;

end