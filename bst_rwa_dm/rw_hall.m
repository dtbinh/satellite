function dat = rw_hall(par, dat, spd, t)

% Update Time Stamp for Old 
    dat.t_spd_m_old   = dat.t_spd_m_cur;
    dat.spd_m_old = dat.spd_m;
    
    % Update Current Speed Measurement
    dat.t_spd_m_cur = t;
    dat.spd_m = (1-par.filter)*spd + par.filter*dat.spd_m;

end