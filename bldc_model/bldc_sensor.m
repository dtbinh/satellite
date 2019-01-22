function out = bldc_sensor(in)
tick_a = in(1);
tick_b = in(2); 
tick_c = in(3);
time   = in(4);

persistent tick_tot t_cur spd spd_old trq;
global CONST;
% Initialise
if isempty(tick_tot)
    tick_tot = 0;
    t_cur = 0;
    spd = 0;
    spd_old = 0;
    trq = 0;
end

interval = 1;
n_filter = 0.0;

if (tick_a)

        t_old = t_cur;
        t_cur = time;
        if (t_cur ~=t_old) 
            dt = t_cur - t_old;
            spd =(1-n_filter)*2*pi/dt + n_filter*spd; % complimentary filter
            trq = CONST.J*(spd - spd_old)/dt;
            spd_old = spd;
        end
end

out(1,1) = spd;
out(2,1) = trq;
end