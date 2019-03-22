function trq = rw_mdl_friction(par, spd, temp, type)
rpm2rps = 2*pi/60;

% fprintf('spd:%.6f [rpm]\n',spd*60/2/pi);

if (strcmp(type,'neg')) && ((spd*par.ke) < par.diode_drop)
    trq = polyval(par.frct_p,spd);

else
    
% friction model only valid until 500 rpm, then we assume linear to zero for now
%     if (abs(spd)<500*rpm2rps)
%         spd = sign(spd)*500*rpm2rps;
%     end
    
    % Friction Model
    switch type
    case 'pos'
        trq = polyval(par.frct_p,abs(spd));
%         fprintf('Positive Friction: %.9f [Nm]\n',trq);
    case 'neg'
        trq = polyval(par.frct_p,abs(spd));
    case 'idle'
        trq = polyval(par.frct_c,abs(spd));

    otherwise
        fprintf('friction type not specified\n');
        trq = polyval(par.frct_c,abs(spd));
    end
end

% Temperature Modeling

if (strcmp(type,'pos'))
    dT = temp - par.temp_ref;
    trq = par.frctn_temp_p(1)*dT*dT + par.frctn_temp_p(2)*dT + trq;
    if(dT)
       fprintf('Temp. modeling dT: %.3f\n',dT); 
    end
elseif (temp ~= par.temp_ref)
    fprintf('Temp. modeling missing\n');
end


% friction model only valid until 500 rpm, 
if (abs(spd)<500*rpm2rps)
   trq =  (abs(spd)/(500*rpm2rps))*trq;
end

if spd < 0.0
   trq = -trq; 
end
   
    
end
