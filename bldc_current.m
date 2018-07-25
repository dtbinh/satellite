function out = bldc_current(in)

V_a = in(1);
V_b = in(2); 
V_c = in(3);

E_a = in(4);
E_b = in(5); 
E_c = in(6);

i_a = in(7);
i_b = in(8); 
i_c = in(9);

L = 9.4e-3; %[H]
R = 1.43;   % [Ohm]


idot_a = 1/L*(V_a-R*i_a-E_a);
idot_b = 1/L*(V_b-R*i_b-E_b);
idot_c = 1/L*(V_c-R*i_c-E_c);

out(1,1) = idot_a;
out(2,1) = idot_b;
out(3,1) = idot_c;

end