function Vout = bldc_discrete_speed(w_tgt,w_dm)
global CONST

dt = CONST.dt;
spd_kp = CONST.spd_kp;
spd_ki = CONST.spd_ki;
spd_kd = CONST.spd_kd;

persistent w_err_old w_err_i_old

if isempty(w_err_old)
   w_err_old = 0; 
   w_err_i_old = 0;
end

w_err = w_tgt-w_dm;

w_err_i = w_err_i_old + w_err*dt;
w_err_d = (w_err-w_err_old)/dt;

Vout = spd_kp*w_err + spd_ki*w_err_i + spd_kd*w_err_d;

w_err_old = w_err;
w_err_i_old = w_err_i;
end