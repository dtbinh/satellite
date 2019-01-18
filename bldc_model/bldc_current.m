function out = bldc_current(in)

% Phase Voltage
V_a = in(1);
V_b = in(2); 
V_c = in(3);

% back-EMF
E_a = in(4);
E_b = in(5); 
E_c = in(6);

global CONST

i_a = 1/CONST.R*(V_a-E_a);
i_b = 1/CONST.R*(V_b-E_b);
i_c = 1/CONST.R*(V_c-E_c);

out(1,1) = i_a;
out(2,1) = i_b;
out(3,1) = i_c;

end