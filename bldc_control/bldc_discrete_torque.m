function Vout = bldc_discrete_torque(trq_tgt,trq_dm)
global CONST

dt = CONST.dt;
trq_kp = CONST.trq_kp;
trq_ki = CONST.trq_ki;
trq_kd = CONST.trq_kd;

persistent trq_err_old trq_err_i_old

if isempty(trq_err_i_old)
   trq_err_old = 0; 
   trq_err_i_old = 0;
end

trq_err   = trq_tgt-trq_dm;
trq_err_i = trq_err_i_old + trq_err*dt;
trq_err_d = (trq_err-trq_err_old)/dt;

Vout = trq_kp*trq_err + trq_ki*trq_err_i+trq_kd*trq_err_d;

trq_err_old   = trq_err;
trq_err_i_old = trq_err_i;
end