function trq = rw_get_friction(RW, spd, type)
RW.diode_drop= 0.250;

if (strcmp(type,'neg')) && ((spd*RW.ke) < RW.diode_drop)
%     fprintf('Neg but its ok\n');
    trq = polyval(RW.frct_p,spd);
else
    
    switch type
    case 'pos'
        trq = polyval(RW.frct_p,spd);
    case 'neg'
        trq = polyval(RW.frct_n,spd);
    case 'idle'
        trq = polyval(RW.frct_c,spd);
    otherwise
        fprintf('frictino type not specified\n');
        trq = polyval(RW.frct_c,spd);
    end

end



if spd < 0.0
   trq = -trq; 
end

if (abs(spd)<500*2*pi/60)
   trq =  (abs(spd)/(500*2*pi/60))*trq;
end
    
    
end
