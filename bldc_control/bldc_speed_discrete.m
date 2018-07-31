function Vout = bldc_speed_discrete(w_tgt,w_dm)
global CONST

dt = CONST.dt;
spd_kp = CONST.spd_kp;
spd_ki = CONST.spd_ki;
spd_kd = CONST.spd_kd;

persistent w_err_old

if isempty(w_err_old)
   w_err_old = 0; 
end

w_err = w_tgt-w_dm;

w_err_i = (w_err-w_err_old)*dt;

Vout = spd_kp*w_err + spd_ki*w_err_i;

w_err_old = w_err;
end