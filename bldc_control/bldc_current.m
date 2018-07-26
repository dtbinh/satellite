function out = bldc_current(in)

V_a = in(1);
V_b = in(2); 
V_c = in(3);

E_a = in(4);
E_b = in(5); 
E_c = in(6);

global CONST

L = CONST.L;
R = CONST.R;   

i_a = 1/R*(V_a-E_a);
i_b = 1/R*(V_b-E_b);
i_c = 1/R*(V_c-E_c);

out(1,1) = i_a;
out(2,1) = i_b;
out(3,1) = i_c;

end