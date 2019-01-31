function trq = rw_get_friction(RW, spd, type)

% fprintf('spd:%.6f [rpm]\n',spd*60/2/pi);

if (strcmp(type,'neg')) && ((spd*RW.ke) < RW.diode_drop)
%      fprintf('Neg but its ok\n');
    trq = polyval(RW.frct_p,spd);
else
    
    switch type
    case 'pos'
        trq = polyval(RW.frct_p,spd);
%         fprintf('pos trq:%.6f\n',trq);
    case 'neg'
        trq = polyval(RW.frct_n,spd);
%         trq = -0.25*RW.km*(RW.ke*spd)/RW.R;
%           fprintf('neg trq:%.6f\n',trq);
    case 'idle'
        trq = polyval(RW.frct_c,spd);
%         fprintf('idle trq:%.6f\n',trq);
    case'ke'
%         trq = -RW.km*(RW.ke*spd)/RW.R;
        trq = polyval(RW.frct_n,spd);
    otherwise
        fprintf('friction type not specified\n');
        trq = polyval(RW.frct_c,spd);
    end
end



if spd < 0.0
   trq = -trq; 
end

% friction model only valid until 500 rpm, 
if (abs(spd)<500*2*pi/60)
   trq =  (abs(spd)/(500*2*pi/60))*trq;
end
    
    
end
