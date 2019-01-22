function Vout = bldc_discrete_current(i_tgt,i_dm)
global CONST

dt = CONST.dt;
i_kp = CONST.i_kp;
i_ki = CONST.i_ki;
i_kd = CONST.i_kd;

persistent i_err_old i_err_i_old

if isempty(i_err_i_old)
   i_err_old = 0; 
   i_err_i_old = 0;
end

i_err   = i_tgt-i_dm;
i_err_i = i_err_i_old + i_err*dt;
i_err_d = (i_err-i_err_old)/dt;

Vout = i_kp*i_err + i_ki*i_err_i+i_kd*i_err_d;

i_err_old   = i_err;
i_err_i_old = i_err_i;
end