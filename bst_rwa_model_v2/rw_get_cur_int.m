function cur_int = rw_get_cur_int(par, spd, cur_tgt, temp, is_via_frctn)

% No Internal Current (Fwd Operation)
if((spd*cur_tgt) >= 0.0)
   cur_int = 0.0;
   return
end

% No Internal Current (Bwd Operation)
if((abs(spd)*par.ke) <= par.diode_drop)
   cur_int = 0.0;
   return
end

% Internal Current Model
if(is_via_frctn) 
    ki = 1/rw_get_km(par,spd,cur_tgt); % [A/Nm]
    cur_int = ki*(rw_get_friction(par,spd,temp,'neg')-rw_get_friction(par,spd,temp,'pos'));
else             
    dT      = temp-par.temp_ref;
    cur_int = (par.ke*abs(spd) - par.diode_drop)/(par.r_mot + par.r_mot_temp*(dT));
    if(spd>0.0)
       cur_int = -1.0*cur_int; 
    end
%     fprintf('dt: %.3f\n',dT);
end

if (abs(cur_tgt)>abs(cur_int))
    cur_int = 0.0;
end
    


% fprintf('Internal Current: %.3f [A]\n',cur_int);
end