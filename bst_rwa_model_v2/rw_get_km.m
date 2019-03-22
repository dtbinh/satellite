function km = rw_get_km(par, spd, cur_tgt)

cur_abs = abs(cur_tgt);

if(cur_abs< par.cur_km(1))
    km = rw_get_km_cx(par,spd,1);
elseif(cur_abs< par.cur_km(2))
    km_ca = rw_get_km_cx(par,spd,1);
    km_cb = rw_get_km_cx(par,spd,2);
    km    = km_ca + (km_cb - km_ca)/(par.cur_km(2) - par.cur_km(1)) * (cur_abs - par.cur_km(1));
    
elseif(cur_abs< par.cur_km(3))
    km_ca = rw_get_km_cx(par,spd,2);
    km_cb = rw_get_km_cx(par,spd,3);
    km    = km_ca + (km_cb - km_ca)/(par.cur_km(3) - par.cur_km(2)) * (cur_abs - par.cur_km(2));
    
else
    km    = rw_get_km_cx(par,spd,3);
end

end
