function out = rw_simulink_model(in) 

ctrl_mode = in(1);
cur_tgt   = in(2); 
spd_tgt   = in(3);
trq_tgt   = in(4); 
vol_in    = in(5);
spd_init  = in(6); 
dt        = in(7);
time      = in(8);
model     = 'fm2';

[trq,spd,trq_m,spd_m,cur_cmd] = rw_model(model,ctrl_mode, cur_tgt, spd_tgt, trq_tgt, vol_in, spd_init, dt, time) ;


out(1)  = spd;
out(2)  = spd_m;
out(3)  = trq;
out(4)  = trq_m;
out(5)  = cur_cmd;

end