function km_cx = rw_get_km_cx(par, spd, idx)


spd_abs = abs(spd);

if(spd_abs < par.km_spd(idx*2-1)) 
    km_cx = par.km_val(idx*2-1);
elseif(spd_abs < par.km_spd(idx*2))
    km_cx = par.km_val(idx*2-1) + (par.km_val(idx*2) - par.km_val(idx*2-1))/(par.km_spd(idx*2) - par.km_spd(idx*2-1)) * (spd_abs - par.km_spd(idx*2-1)); 
else
    km_cx = par.km_val(idx*2); 
end

end